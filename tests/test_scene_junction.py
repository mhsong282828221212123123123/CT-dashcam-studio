import unittest
from types import SimpleNamespace
import cv2,numpy as np
from core.scene_junction import junction_evidence,JunctionTracker,apply_camera_junctions
from core.scene_fusion import CameraGeometry
from core.driving_scene import Actor


class JunctionTests(unittest.TestCase):
    def image(self,open_sides=True):
        image=np.full((480,640,3),90,np.uint8)
        if not open_sides:image[:, :160]=[20,100,20];image[:,480:]=[20,100,20]
        for x in (80,280,480):cv2.rectangle(image,(x,320),(x+50,345),(220,220,220),-1)
        return image

    def test_markings_and_side_road_are_both_required(self):
        self.assertTrue(junction_evidence(self.image())['candidate'])
        self.assertFalse(junction_evidence(self.image(False))['candidate'])
        self.assertFalse(junction_evidence(np.full((480,640,3),90,np.uint8))['candidate'])

    def test_object_paint_is_excluded(self):
        self.assertFalse(junction_evidence(self.image(),[{'bbox':[60,300,500,60]}])['candidate'])

    def test_temporal_support_and_gap_reset(self):
        tracker=JunctionTracker();e=junction_evidence(self.image())
        self.assertFalse(tracker.update(e,0)['active'])
        self.assertFalse(tracker.update(e,.1)['active'])
        for i in range(2,10):result=tracker.update(e,i*.1)
        self.assertTrue(result['active'])
        self.assertFalse(tracker.update(e,2)['active'])

    def test_no_map_required_to_open_observed_junction_band(self):
        e=dict(junction_evidence(self.image()),active=True)
        scene=SimpleNamespace(junction_evidence=e,road_surfaces=[],
            lanes=[{'points':[[0,float(z)] for z in np.arange(0,30,.25)]}])
        audit={};apply_camera_junctions([scene],[{'front':CameraGeometry(width=640,height=480,focal=425,horizon=237)}],[audit])
        self.assertTrue(scene.road_surfaces)
        self.assertEqual(audit['camera_junction']['source'],'camera_only_no_map')
        lower,upper=audit['camera_junction']['longitudinal_band_m']
        self.assertTrue(all(not lower<=p[1]<=upper for lane in scene.lanes for p in lane['points']))

    def test_transverse_vehicle_support_compensates_host_motion(self):
        def run(host_moves):
            scenes=[];poses=[];audits=[{},{}]
            for i,t in enumerate([0.,.6]):
                host=np.array([2.*i,0.]) if host_moves else np.zeros(2)
                actors=[Actor(str(k),'car',float(k*4+(0 if host_moves else i*2)-host[0]),20.) for k in range(2)]
                scenes.append(SimpleNamespace(time=t,actors=actors,road_surfaces=[],lanes=[],
                    junction_evidence=dict(junction_evidence(self.image()),active=True)))
                poses.append((host,np.eye(2)))
            apply_camera_junctions(scenes,[{'front':CameraGeometry(width=640,height=480,focal=425,horizon=237)}]*2,audits,poses)
            return audits[1]['camera_junction']['transverse_vehicle_support']
        self.assertEqual(run(True),0)
        self.assertEqual(run(False),2)
