import copy
import unittest
import numpy as np
from ct_scene.context_surface import build,registration,warp_context_points,fit_display_shift


class ContextSurfaceTests(unittest.TestCase):
    def short_anchors(self):
        return [dict(normal=np.array([0.,1.]),target=17.+i*.03,
                     position=np.array([i*1.1,0.]),time_s=float(i)) for i in range(9)]

    def test_short_stable_approach_fits_lateral_but_not_longitudinal(self):
        anchors=self.short_anchors()
        self.assertIsNone(fit_display_shift(anchors))
        result=fit_display_shift(anchors,allow_short=True)
        self.assertTrue(result['accepted']);self.assertEqual(result['support_kind'],'short_stable_lateral_only')
        self.assertFalse(result['longitudinal_correction_fitted'])
        self.assertAlmostEqual(result['translation'][0],0.)

    def test_short_rule_does_not_accept_stationary_noise_or_turns(self):
        anchors=self.short_anchors()
        for a in anchors:a['position']*=0
        self.assertIsNone(fit_display_shift(anchors,allow_short=True))
        anchors=self.short_anchors();anchors[-1]['target']+=2
        self.assertIsNone(fit_display_shift(anchors,allow_short=True))
        anchors=self.short_anchors();anchors[-1]['normal']=np.array([.2,np.sqrt(.96)])
        self.assertIsNone(fit_display_shift(anchors,allow_short=True))

    def data(self):
        rows=[]
        for i in range(10):
            fields={k:dict(valid=True,value=v) for k,v in [('latitude_deg',37.+i*.00004),('longitude_deg',127.),('heading_deg',0.)]}
            rows.append(dict(camera_time_s=float(i),fields=fields))
        ways=[dict(id=1,geometry=[[37.,127.],[37.001,127.]],tags=dict(highway='primary')),
              dict(id=2,geometry=[[37.,126.9995],[37.,127.0005]],tags=dict(highway='primary',layer='1',bridge='yes'))]
        t=np.arange(10,dtype=float);ego=np.c_[t*4.4528,t*0,t*0]
        records=[dict(way_candidates=[dict(way_id=1)])]*4
        return ways,rows,t,ego,records

    def test_map_width_and_lane_tags_never_assign_geometry_or_move_ego(self):
        ways,rows,t,ego,records=self.data();before=ego.copy()
        original=build(ways,rows,t,ego,records)
        changed=copy.deepcopy(ways)
        for w in changed:w['tags'].update(lanes='99',width='1000')
        self.assertEqual(original,build(changed,rows,t,ego,records))
        self.assertEqual(original['way_ids'],[1]);np.testing.assert_array_equal(ego,before)
        self.assertFalse(original['scale_fitted'])

    def test_rigid_registration_preserves_scale(self):
        gps=np.c_[np.arange(10)*5.,np.zeros(10)]
        expected=np.array([[0.,1.],[-1.,0.]])
        world=gps@expected+[4.,7.]
        R,offset,error=registration(gps,world,0.)
        np.testing.assert_allclose(gps@R+offset,world,atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(R),1.);self.assertLess(error,1e-10)

    def test_missing_gps_abstains(self):
        ways,rows,t,ego,records=self.data()
        self.assertEqual(build(ways,rows[:2],t,ego,records)['polygons_W_xy'],[])

    def test_offset_main_road_is_fused_not_rendered_as_parallel_road(self):
        ways,rows,t,ego,records=self.data()
        # GPS says the road is about nine metres east of its visual position.
        ways[0]['geometry']=[[37.,127.0001],[37.001,127.0001]]
        ways.append(dict(id=3,geometry=[[37.,126.9998],[37.001,126.9998]],tags=dict(highway='residential')))
        records=[dict(time_s=float(i),way_candidates=[dict(way_id=1)]) for i in range(10)]
        road=dict(boundaries_y=[5.25,1.75,-1.75]) # ego is in the right lane
        before=ego.copy();result=build(ways,rows,t,ego,records,road)
        self.assertTrue(result['main_road_alignment']['accepted'])
        self.assertEqual(result['suppressed_duplicate_main_way_ids'],[1])
        self.assertEqual(result['way_ids'],[3]) # the genuine neighbouring road remains
        self.assertEqual(result['main_road_alignment']['target'],'camera_road_centre_not_ego_lane')
        self.assertLess(result['main_road_alignment']['normal_residual_p90_m'],.01)
        np.testing.assert_array_equal(ego,before)

    def test_stationary_or_unmatched_map_does_not_create_a_false_side_road(self):
        ways,rows,t,ego,records=self.data()
        records=[dict(time_s=float(i),way_candidates=[dict(way_id=1)]) for i in range(10)]
        result=build(ways,rows,t,ego*0,records,dict(boundaries_y=[2,-2]))
        self.assertEqual(result['status'],'map_context_alignment_unresolved')
        self.assertEqual(result['polygons_W_xy'],[])

    def test_main_road_continuation_is_fused_even_without_local_lane_support(self):
        ways,rows,t,ego,records=self.data()
        ways.append(dict(id=3,geometry=[[37.0003,127.],[37.001,127.]],tags=dict(highway='primary')))
        records=[dict(time_s=float(i),way_candidates=[dict(way_id=1 if i<8 else 3)]) for i in range(10)]
        road=dict(boundaries_y=[2,-2],reference_line=dict(points_W_xy=[[0,0],[50,0]],supported_extent_m=[0,32]))
        result=build(ways,rows,t,ego,records,road)
        self.assertTrue(result['main_road_alignment']['accepted'])
        self.assertEqual(result['suppressed_duplicate_main_way_ids'],[1,3])
        self.assertEqual(result['polygons_W_xy'],[])

    def test_changing_map_bias_uses_local_road_support_without_moving_ego(self):
        ways,rows,t,ego,records=self.data()
        rows=[]
        for i in range(20):
            rows.append(dict(camera_time_s=float(i),fields={k:dict(valid=True,value=v) for k,v in
                [('latitude_deg',37.+i*.00004),('longitude_deg',127.),('heading_deg',0.)]}))
        t=np.arange(20,dtype=float);ego=np.c_[t*4.4528,t*0,t*0];before=ego.copy()
        ways=[dict(id=1,geometry=[[37.,127.0001],[37.0004,127.0001]],tags=dict(highway='primary')),
              dict(id=3,geometry=[[37.0004,126.9999],[37.0009,126.9999]],tags=dict(highway='primary')),
              dict(id=4,geometry=[[37.0002,127.0001],[37.0002,127.0003]],tags=dict(highway='residential'))]
        records=[dict(time_s=float(i),way_candidates=[dict(way_id=1 if i<10 else 3)]) for i in range(20)]
        result=build(ways,rows,t,ego,records,dict(boundaries_y=[2,-2]))
        alignment=result['main_road_alignment']
        self.assertTrue(alignment['accepted']);self.assertFalse(alignment['global_alignment_accepted'])
        self.assertEqual(len(alignment['local_regions']),2)
        self.assertEqual(result['way_ids'],[4]);self.assertTrue(result['polygons_W_xy'])
        self.assertEqual(result['suppressed_duplicate_main_way_ids'],[1,3])
        np.testing.assert_array_equal(ego,before)

    def test_spatial_correction_preserves_shared_junction_nodes(self):
        regions=[dict(map_anchor_positions_W_xy=[[0,0],[20,0]],display_translation_W_xy=[0,8]),
                 dict(map_anchor_positions_W_xy=[[40,0],[60,0]],display_translation_W_xy=[0,-8])]
        # Two ways sharing a node must get the same correction independent of
        # vertex order, instead of pulling the junction into two locations.
        a,_=warp_context_points([[0,0],[30,0]],regions)
        b,_=warp_context_points([[60,0],[30,0]],regions)
        np.testing.assert_allclose(a[-1],b[-1])
        self.assertLessEqual(abs(a[-1,1]),8.)

    def test_dual_carriageway_opposing_way_is_detected_and_suppressed(self):
        ways,rows,t,ego,records=self.data()
        # Main way is northbound oneway=yes secondary; way 3 is southbound ~10m to its left (west); way 4 is a cross street
        ways=[dict(id=1,geometry=[[37.,127.],[37.001,127.]],tags=dict(highway='secondary',oneway='yes',name='TestRd')),
              dict(id=3,geometry=[[37.001,126.99988],[37.,126.99988]],tags=dict(highway='secondary',oneway='yes',name='TestRd')),
              dict(id=4,geometry=[[37.0002,126.9997],[37.0002,127.0003]],tags=dict(highway='residential'))]
        records=[dict(time_s=float(i),way_candidates=[dict(way_id=1)]) for i in range(10)]
        road=dict(boundaries_y=[3.5,0.,-3.5])
        result=build(ways,rows,t,ego,records,road)
        self.assertTrue(result['main_way_dual_carriageway'])
        self.assertTrue(result['main_way_explicit_oneway'])
        self.assertEqual(result['suppressed_opposing_way_ids'],[3])
        self.assertEqual(result['way_ids'],[4])
        # Pure motorway must never be flagged as a dual-carriageway ordinary road
        mw_ways=copy.deepcopy(ways)
        mw_ways[0]['tags']['highway']='motorway';mw_ways[1]['tags']['highway']='motorway'
        mw_res=build(mw_ways,rows,t,ego,records,road)
        self.assertFalse(mw_res['main_way_dual_carriageway'])
        self.assertEqual(mw_res['suppressed_opposing_way_ids'],[])
