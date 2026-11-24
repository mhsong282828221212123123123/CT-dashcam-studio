import unittest
import numpy as np
from core.scene_fusion import cameras_for,default_profile,corners,dimensions
from core.scene_truck_shapes import infer_truck_shapes


class TruckShapeTests(unittest.TestCase):
    def fixture(self,kind):
        cameras=cameras_for({'cameras':{'front':{'width':1280,'height':960}}},default_profile())
        rows=[]
        for i in range(20):
            bounds=cameras['front'].visible_bounds(corners(3.,18+i*.5,dimensions(kind),kind=kind))
            box=np.r_[bounds[:2],bounds[2:]-bounds[:2]].tolist()
            rows.append({'time_s':i*.1,'cameras':{'front':{'source_time_s':i*.1,
                'objects':[{'kind':'truck','track_status':'confirmed','track_id':'front:1','bbox':box}]}}})
        return rows,[cameras]*len(rows)

    def test_tall_truck_gets_one_shape_from_multiple_distances(self):
        result=infer_truck_shapes(*self.fixture('truck_medium'))
        self.assertEqual(result['front:1']['shape'],'truck_medium')
        self.assertFalse(result['front:1']['dimensions_verified'])

    def test_low_cargo_retains_default_and_sparse_evidence_cannot_select(self):
        self.assertEqual(infer_truck_shapes(*self.fixture('truck')), {})
        rows,cameras=self.fixture('truck_heavy')
        self.assertEqual(infer_truck_shapes(rows[:3],cameras[:3]),{})

    def test_explicit_open_bed_geometry_is_not_replaced_by_box_size_selection(self):
        rows,cameras=self.fixture('truck_medium')
        for row in rows:row['cameras']['front']['objects'][0]['geometry_kind']='small_truck'
        self.assertEqual(infer_truck_shapes(rows,cameras),{})

    def test_open_bed_projection_distinguishes_near_cab_from_far_cab(self):
        import math
        camera=cameras_for({'cameras':{'front':{'width':1280,'height':960}}},default_profile())['front']
        rear=camera.visible_bounds(corners(0,20,dimensions('small_truck'),kind='small_truck'))
        front=camera.visible_bounds(corners(0,20,dimensions('small_truck'),kind='small_truck',yaw=math.pi))
        self.assertLess(front[1],rear[1]-5)
        self.assertLess(abs(front[3]-rear[3]),2.)
