import math
import unittest
import numpy as np
from core.scene_fusion import (CameraGeometry,Candidate,FusedSceneTimeline,corners,dimensions,
                               fit_box,fuse_candidates,default_profile,cameras_for,joint_position)


def detection(key,box,kind='car'):
    return {'track_id':key,'bbox':list(box),'kind':kind,'confidence':.9,'track_status':'confirmed'}


def frame(t,cameras):
    return {'time_s':t,'ego':{'data':{'speed_kmh':0}},'cameras':{
        camera:{'available':True,'objects':objects,'source_time_s':t} for camera,objects in cameras.items()}}


class SceneFusionTests(unittest.TestCase):
    def test_forward_contact_is_not_sacrificed_to_representative_body_size(self):
        # Real user clip, 5 s: equal relative edge weights left the fitted tyres
        # ~15 px above the detected bottom and put the truck at x=5.255 m.
        camera=CameraGeometry(horizon=471.2864041083843)
        box=[223.03258666992187,424.7377990722656,199.06021728515626,148.53812255859373]
        fit=fit_box(camera,box,'truck')
        self.assertIsNotNone(fit)
        bounds=camera.visible_bounds(corners(*fit[0],fit[1],kind='truck'))
        self.assertLess(abs(bounds[3]-(box[1]+box[3])),1.)
        # This is an image-contact check, not surveyed vehicle-position truth.

    def test_contact_precision_does_not_amplify_distant_or_repeater_boxes(self):
        from core.scene_fusion import box_fit_scales
        for camera,w,h in [(CameraGeometry(),40,30),
                           (CameraGeometry(yaw=math.radians(146)),200,150)]:
            np.testing.assert_array_equal(box_fit_scales(camera,w,h),
                                          box_fit_scales(camera,w,h,weighted=False))

    def test_contact_priority_keeps_old_supported_detection_when_contact_conflicts(self):
        from unittest.mock import patch
        camera=CameraGeometry()
        result=(np.array([3.5,15.]),dimensions('car'),.12)
        with patch('core.scene_fusion._fit_box',side_effect=[None,result]) as fit:
            self.assertIs(fit_box(camera,[200,400,150,150],'car'),result)
        self.assertEqual(fit.call_args.kwargs,{'prioritize_contact':False,'yaw':0.})

    def test_exact_forward_boxes_remain_unbiased_with_contact_priority(self):
        camera=CameraGeometry()
        for x,z,kind in [(-3.5,12,'car'),(3.5,18,'truck'),(0,35,'car')]:
            bounds=camera.visible_bounds(corners(x,z,dimensions(kind),kind=kind))
            box=np.r_[bounds[:2],bounds[2:]-bounds[:2]]
            fitted=fit_box(camera,box,kind)
            self.assertIsNotNone(fitted)
            np.testing.assert_allclose(fitted[0],[x,z],atol=.04)

    def test_motorcycle_body_box_does_not_shrink_rendered_rider(self):
        manifest={'sample_hz':10,'cameras':{'front':{'width':1280,'height':960}}}
        camera=cameras_for(manifest,default_profile())['front']
        bounds=camera.visible_bounds(corners(3,18,dimensions('motorcycle_body'),kind='motorcycle_body'))
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]]
        rows=[frame(i*.1,{'front':[detection('front:bike',box,'motorcycle')]}) for i in range(5)]
        timeline=FusedSceneTimeline(rows,manifest)
        actor=timeline.scenes[-1].actors[0]
        self.assertEqual(actor.geometry_kind,'motorcycle_body')
        self.assertEqual(actor.height,dimensions('motorcycle')[2])
        np.testing.assert_allclose([actor.x,actor.z],[3,18],atol=.1)

    def test_truck_envelope_does_not_use_sedan_sloping_roof(self):
        size=dimensions('truck')
        points=corners(0,0,size,kind='truck')
        roof=points[np.isclose(points[:,1],size[2])]
        self.assertAlmostEqual(np.ptp(roof[:,0]),size[0])
        self.assertAlmostEqual(np.ptp(roof[:,2]),size[1])

    def test_offline_handoffs_are_enabled_and_can_be_disabled(self):
        from unittest.mock import patch
        rows=[frame(0,{'front':[]}),frame(.1,{'front':[]})]
        manifest={'sample_hz':10,'cameras':{'front':{'width':1280,'height':960}}}
        with patch('core.scene_handoff.stitch_handoffs',return_value=17) as stitch:
            timeline=FusedSceneTimeline(rows,manifest)
            self.assertEqual(timeline.stitched_handoffs,17)
            stitch.assert_called_once()
        with patch('core.scene_handoff.stitch_handoffs') as stitch:
            self.assertEqual(FusedSceneTimeline(rows,manifest,enable_handoffs=False).stitched_handoffs,0)
            stitch.assert_not_called()

    def test_long_gap_does_not_restore_stale_road_or_signals(self):
        from core.driving_scene import Scene
        timeline=FusedSceneTimeline.__new__(FusedSceneTimeline)
        timeline.max_gap=.2
        first=Scene(0,None,lanes=[{'points':[[0,5],[0,20]]}])
        first.signals=[{'state':'green'}]
        first.road_surface=[[-2,0],[2,0],[2,20],[-2,20]]
        timeline.scenes=[first,Scene(2,None)]
        middle=timeline.sample(1.)
        self.assertEqual(middle.lanes,[])
        self.assertEqual(getattr(middle,'signals',[]),[])
        self.assertEqual(getattr(middle,'road_surface',[]),[])

    def test_visible_approaching_repeater_car_is_not_above_default_horizon(self):
        # Actual HW3 right-repeater observation at 2026-09-05 14:54:24.
        # This clip was not used to estimate the revised horizon prior.
        manifest={'sample_hz':10,'cameras':{'right_repeater':{'width':1280,'height':960}}}
        box=[902.8697814941406,439.788818359375,170.41131591796875,105.90576171875]
        camera=cameras_for(manifest,default_profile())['right_repeater']
        result=fit_box(camera,box,'car')
        self.assertIsNotNone(result)
        self.assertLess(result[0][0],0)
        self.assertLess(result[0][1],0)
        rows=[frame(i*.1,{'right_repeater':[detection('right_repeater:1',box)]}) for i in range(5)]
        timeline=FusedSceneTimeline(rows,manifest)
        self.assertTrue(all(any('right_repeater:1' in a.sources for a in s.actors) for s in timeline.scenes))

    def test_detected_truck_is_not_silently_rendered_as_sedan(self):
        manifest={'sample_hz':10,'cameras':{'front':{'width':1280,'height':960}}}
        camera=cameras_for(manifest,default_profile())['front']
        bounds=camera.visible_bounds(corners(0,12,dimensions('truck'),kind='truck'))
        box=[bounds[0],bounds[1],bounds[2]-bounds[0],bounds[3]-bounds[1]]
        timeline=FusedSceneTimeline([frame(i*.1,{'front':[detection('front:1',box,'truck')]}) for i in range(5)],manifest)
        self.assertEqual(timeline.scenes[-1].actors[0].kind,'truck')

    def test_rear_cannot_pull_primary_position_or_seed_position(self):
        primary=Candidate('left_repeater','left:1','car',3.5,-10,.6,dimensions('car'),.15)
        rear=Candidate('back','back:1','bus',-20,-50,.99,dimensions('bus'),.001)
        np.testing.assert_array_equal(joint_position([primary,rear],{}),[3.5,-10])
        self.assertIsNone(joint_position([rear],{}))

    def test_near_plane_clips_vehicle_instead_of_discarding_visible_half(self):
        camera=CameraGeometry(focal=650,horizon=300,yaw=math.pi,camera_height=.8,z=-2.2)
        points=corners(3.93,-3.5,dimensions('suv'),kind='suv')
        self.assertIsNone(camera.pixels(points))
        bounds=camera.visible_bounds(points)
        self.assertIsNotNone(bounds);self.assertTrue(np.isfinite(bounds).all())
        self.assertTrue(0<=bounds[0]<bounds[2]<=1280)
        self.assertTrue(0<=bounds[1]<bounds[3]<=960)
        self.assertIsNone(camera.visible_bounds(corners(0,10,dimensions('car'))))

    def test_partial_observation_requires_temporal_multiview_support_and_cannot_seed(self):
        manifest={'sample_hz':10,'cameras':{c:{'width':1280,'height':960} for c in ('back','left_repeater')}}
        profile=default_profile();profile['cameras']['left_repeater']['horizon']=502
        cameras=cameras_for(manifest,profile);objects={}
        for name,camera in cameras.items():
            b=camera.visible_bounds(corners(3.93,-3.5,dimensions('suv'),kind='suv'))
            obj=detection(name+':1',np.r_[b[:2],b[2:]-b[:2]])
            obj['render_kind']='suv'
            if name=='back':obj['association_only']=True
            objects[name]=[obj]
        rows=[frame(i*.1,objects) for i in range(6)]
        timeline=FusedSceneTimeline(rows,manifest,profile)
        self.assertEqual(timeline.at(0).fusion_count,0)
        self.assertEqual(timeline.at(3).fusion_count,1)
        self.assertEqual(len(timeline.at(3).actors),1)
        self.assertEqual(set(timeline.at(3).actors[0].sources),{'back:1','left_repeater:1'})
        self.assertEqual(len({s.actors[0].key for s in timeline.scenes}),1)
        alone=FusedSceneTimeline([frame(i*.1,{'back':objects['back']}) for i in range(6)],manifest,profile)
        self.assertTrue(all(not s.actors for s in alone.scenes))
        wrong=dict(objects['back'][0],bbox=[0,180,260,375])
        wrong_rows=[frame(i*.1,{'back':[wrong],'left_repeater':objects['left_repeater']}) for i in range(6)]
        rejected=FusedSceneTimeline(wrong_rows,manifest,profile)
        self.assertTrue(all(s.fusion_count==0 for s in rejected.scenes))
        no_rear=FusedSceneTimeline([frame(i*.1,{'left_repeater':objects['left_repeater']}) for i in range(6)],manifest,profile)
        for supported,unsupported in zip(timeline.scenes,no_rear.scenes):
            a,b=supported.actors[0],unsupported.actors[0]
            self.assertEqual((a.key,a.x,a.z,a.confidence,a.width,a.length,a.position_uncertainty),
                             (b.key,b.x,b.z,b.confidence,b.width,b.length,b.position_uncertainty))

    def test_offline_occlusion_bridge_is_labeled_and_does_not_invent_sources(self):
        camera=CameraGeometry();pixels=camera.pixels(corners(3.5,15,dimensions('car')))
        lo,hi=pixels.min(0),pixels.max(0);obj=detection('front:1',np.r_[lo,hi-lo])
        manifest={'sample_hz':5,'cameras':{'front':{'width':1280,'height':960}}}
        rows=[frame(i*.2,{'front':[] if i in (3,4) else [obj]}) for i in range(7)]
        timeline=FusedSceneTimeline(rows,manifest)
        for i in (3,4):
            actor=timeline.at(i).actors[0]
            self.assertEqual(actor.tracking_state,'interpolated');self.assertEqual(actor.sources,())
            self.assertEqual(actor.key,timeline.at(2).actors[0].key)
            np.testing.assert_allclose([actor.x,actor.z],[3.5,15],atol=.05)
        self.assertTrue(any(e['reason']=='offline two-sided reconstruction' for e in timeline.audit[3]['world_track_events']))

    def test_long_occlusion_is_not_filled_using_distant_future(self):
        camera=CameraGeometry();p=camera.pixels(corners(3.5,15,dimensions('car')))
        lo,hi=p.min(0),p.max(0);obj=detection('front:1',np.r_[lo,hi-lo])
        rows=[frame(i*.2,{'front':[obj] if i<3 or i==14 else []}) for i in range(15)]
        timeline=FusedSceneTimeline(rows,{'sample_hz':5,'cameras':{'front':{'width':1280,'height':960}}})
        self.assertEqual(timeline.at(12).actors,[])

    def test_moving_horizon_uses_lane_evidence_with_explicit_provenance(self):
        rows=[frame(i*.2,{'front':[]}) for i in range(3)]
        for row in rows:
            row['ego']['data'].update(speed_kmh=18,heading=0)
            row['cameras']['front']['lanes']=[{'points':[[100,900],[400,650]],'source':'test'},
                {'points':[[1180,900],[880,650]],'source':'test'}]
        timeline=FusedSceneTimeline(rows,{'sample_hz':5,'cameras':{'front':{'width':1280,'height':960}}})
        self.assertLess(abs(timeline.camera_frames[-1]['front'].horizon-450),1)
        self.assertEqual(timeline.audit[-1]['front_horizon_source'],'observed_lane_pair')

    def test_single_curved_boundary_cannot_rewrite_recovery_pitch_after_a_valid_pair(self):
        rows=[frame(i*.1,{'front':[]}) for i in range(20)]
        for i,row in enumerate(rows):
            row['ego']['data'].update(speed_kmh=18,heading=0)
            row['cameras']['front']['lanes']=[dict(points=[[100,900],[400,650]],source='test')]
            if i<5:row['cameras']['front']['lanes'].append(dict(points=[[1180,900],[880,650]],source='test'))
            else:row['cameras']['front']['lanes'][0]['points']=[[100,900],[520,550]]
        timeline=FusedSceneTimeline(rows,dict(sample_hz=10,cameras=dict(front=dict(width=1280,height=960))))
        self.assertAlmostEqual(timeline.pitch_safe_frames[4]['front'].horizon,timeline.pitch_safe_frames[-1]['front'].horizon)
        self.assertEqual(timeline.audit[-1]['front_horizon_source'],'single_lane_with_previous_road_direction')

    def test_failed_single_line_pitch_uses_held_pitch_as_ghost_without_losing_identity(self):
        camera=CameraGeometry();bounds=camera.visible_bounds(corners(0,15,dimensions('car'),kind='car'))
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist()
        rows=[frame(i*.1,{'front':[detection('front:lead',box)]}) for i in range(20)]
        for i,row in enumerate(rows):
            row['ego']['data'].update(speed_kmh=18,heading=0)
            row['cameras']['front']['lanes']=[] if i<5 else [dict(points=[[100,900],[400,628]],source='test')]
        timeline=FusedSceneTimeline(rows,dict(sample_hz=10,cameras=dict(front=dict(width=1280,height=960))))
        self.assertTrue(all(len(scene.actors)==1 for scene in timeline.scenes))
        self.assertEqual(len({scene.actors[0].key for scene in timeline.scenes}),1)
        self.assertTrue(any(scene.actors[0].tracking_state=='pitch_held' for scene in timeline.scenes))
        self.assertTrue(all(scene.actors[0].image_observed for scene in timeline.scenes))
        self.assertAlmostEqual(timeline.pitch_safe_frames[-1]['front'].horizon,474.)

    def test_camera_dropout_keeps_real_ghost_without_camera_presence(self):
        camera=CameraGeometry();bounds=camera.visible_bounds(corners(0,15,dimensions('car'),kind='car'))
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist()
        rows=[frame(i*.1,{'front':[detection('front:lead',box)] if i<8 else []}) for i in range(12)]
        timeline=FusedSceneTimeline(rows,dict(sample_hz=10,cameras=dict(front=dict(width=1280,height=960))))
        self.assertTrue(timeline.scenes[6].actors[0].image_observed)
        self.assertEqual(timeline.scenes[9].actors[0].tracking_state,'predicted')
        self.assertFalse(timeline.scenes[9].actors[0].image_observed)

    def test_visible_identity_stays_opaque_when_only_blind_metric_prediction_survives(self):
        camera=CameraGeometry();bounds=camera.visible_bounds(corners(0,15,dimensions('car'),kind='car'))
        box=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist()
        rows=[frame(i*.1,{'front':[detection('front:lead',box)]}) for i in range(10)]
        rows[-1]['cameras']['front']['objects'][0]['bbox'][2]*=.3
        timeline=FusedSceneTimeline(rows,dict(sample_hz=10,cameras=dict(front=dict(width=1280,height=960))))
        actor=timeline.scenes[-1].actors[0]
        self.assertEqual(actor.tracking_state,'predicted')
        self.assertEqual(actor.sources,())
        self.assertTrue(actor.image_observed)

    def test_joint_fit_rejects_incompatible_image_evidence_even_if_centers_are_close(self):
        cams={'a':CameraGeometry(),'b':CameraGeometry(x=.7)};group=[]
        for name,cam in cams.items():
            p=cam.pixels(corners(3.5 if name=='a' else -6,15,dimensions('car')))
            lo,hi=p.min(0),p.max(0)
            group.append(Candidate(name,name,'car',3.5,15,.9,dimensions('car'),.02,bbox=tuple(np.r_[lo,hi-lo])))
        self.assertIsNone(joint_position(group,cams))

    def test_projection_roundtrip_preserves_left_right_and_ground_distance(self):
        for yaw in (0,math.pi,math.radians(146),math.radians(-145)):
            camera=CameraGeometry(yaw=yaw)
            p=camera.origin+camera.rotation@np.array([1.,-camera.camera_height,10.])
            p[1]=0
            pixel=camera.pixels([p])
            np.testing.assert_allclose(camera.ground(*pixel[0]),p[[0,2]],atol=1e-8)

    def test_fit_recovers_vehicle_center_not_contact_point(self):
        camera=CameraGeometry()
        expected=np.array([3.5,13.])
        pixel=camera.pixels(corners(*expected,dimensions('car')))
        lo,hi=pixel.min(0),pixel.max(0)
        result=fit_box(camera,[*lo,*(hi-lo)],'car')
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0],expected,atol=.04)

    def test_cross_camera_duplicates_merge_but_same_camera_and_adjacent_lanes_do_not(self):
        def c(camera,key,x,z):
            return Candidate(camera,key,'car',x,z,.9,(1.9,4.7,1.65),.05)
        candidates=[c('back','b1',3.5,-10),c('left_repeater','l1',3.6,-10.2),c('back','b2',0,-10)]
        groups=fuse_candidates(candidates)
        self.assertEqual(sorted(map(len,groups)),[1,2])
        for group in groups: self.assertEqual(len({c.camera for c in group}),len(group))
        ambiguous=fuse_candidates(candidates+[c('back','b3',3.6,-10.1)])
        self.assertEqual(sorted(map(len,ambiguous)),[1,1,1,1])

    def test_real_queue_layout_regression_all_cameras_and_occluded_contact(self):
        # Image-reviewed 3.2s queue: center lead, adjacent left/right queues,
        # left-repeater SUV behind ego. No GPS, video or user identifiers in fixture.
        front=[detection('front:2',[20,447,330,212]),detection('front:1',[805,469,179,124]),
               detection('front:3',[0,512,132,265]),detection('front:4',[405,445,414,310]),
               detection('front:6',[295,427,152,148],'truck'),detection('front:5',[778,467,73,76])]
        cams={'front':front,'back':[],'left_repeater':[detection('left_repeater:1',[321,295,851,465],'truck')],'right_repeater':[]}
        manifest={'sample_hz':5,'cameras':{c:{'width':1280,'height':960} for c in cams}}
        timeline=FusedSceneTimeline([frame(0,cams),frame(.2,cams)],manifest)
        scene=timeline.at(0)
        by_source={source:actor for actor in scene.actors for source in actor.sources}
        self.assertEqual(set(scene.coverage),set(cams))
        self.assertEqual(set(scene.coverage.values()),{'ok'})
        self.assertLess(abs(by_source['front:4'].x),.5)
        self.assertGreater(by_source['front:2'].x,2.5)
        self.assertLess(by_source['front:1'].x,-2.5)
        self.assertGreater(by_source['left_repeater:1'].x,2.5)
        self.assertLess(by_source['left_repeater:1'].z,-1)
        self.assertNotIn('front:5',by_source)
        self.assertGreater(by_source['front:6'].z-by_source['front:2'].z,4)
        for i,a in enumerate(scene.actors):
            for b in scene.actors[i+1:]:
                self.assertFalse(abs(a.x-b.x)<(a.width+b.width)/2 and abs(a.z-b.z)<(a.length+b.length)/2)
        self.assertEqual([(a.key,a.x,a.z) for a in scene.actors],[(a.key,a.x,a.z) for a in timeline.at(1).actors])

    def test_camera_handoff_retains_global_id_and_sync_mismatch_is_excluded(self):
        manifest={'sample_hz':5,'cameras':{c:{'width':1280,'height':960} for c in ['left_pillar','left_repeater']}}
        profile=default_profile();profile['cameras']['left_pillar']=dict(profile['cameras']['back'])
        cameras=cameras_for(manifest,profile)
        observed={}
        for name,camera in cameras.items():
            pixels=camera.pixels(corners(3.5,-10,dimensions('car')))
            lo,hi=pixels.min(0),pixels.max(0)
            observed[name]=[detection(name+':1',np.r_[lo,hi-lo])]
        rows=[frame(0,{'left_pillar':observed['left_pillar']}),frame(.2,observed),frame(.4,{'left_repeater':observed['left_repeater']})]
        timeline=FusedSceneTimeline(rows,manifest,profile)
        self.assertEqual([len(s.actors) for s in timeline.scenes],[1,1,1])
        self.assertEqual(len({s.actors[0].key for s in timeline.scenes}),1)
        self.assertEqual(timeline.at(1).fusion_count,1)
        np.testing.assert_allclose([timeline.at(1).actors[0].x,timeline.at(1).actors[0].z],[3.5,-10],atol=.1)
        rows[1]['cameras']['left_pillar']['source_time_s']=1.
        late=FusedSceneTimeline(rows,manifest,profile)
        self.assertEqual(late.at(1).coverage['left_pillar'],'out of sync')

    def test_camera_transfer_without_simultaneous_overlap_retains_id(self):
        manifest={'sample_hz':5,'cameras':{c:{'width':1280,'height':960} for c in ['left_pillar','left_repeater']}}
        profile=default_profile();profile['cameras']['left_pillar']=dict(profile['cameras']['back'])
        cameras=cameras_for(manifest,profile)
        rows=[]
        for t,(name,camera) in zip([0,.2],cameras.items()):
            pixels=camera.pixels(corners(3.5,-10,dimensions('car')))
            lo,hi=pixels.min(0),pixels.max(0)
            rows.append(frame(t,{name:[detection(name+':1',np.r_[lo,hi-lo])]}))
        timeline=FusedSceneTimeline(rows,manifest,profile)
        self.assertEqual(timeline.at(0).actors[0].key,timeline.at(1).actors[0].key)
