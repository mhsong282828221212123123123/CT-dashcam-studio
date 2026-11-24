import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from ct_scene.observed_surface import frame_cells,extract,rectangles,surface_mesh


PROFILE=dict(model='SIMPLE_RADIAL',params=[600.,320.,240.,0.])
MOUNT=dict(height_m=1.3,yaw_deg=0.,offset_W_ego_m=[0.,0.,0.])


class ObservedSurfaceTests(unittest.TestCase):
    def test_vehicle_boxes_never_create_pavement_and_masks_are_unchanged(self):
        mask=np.zeros((480,640),np.uint8);before=mask.copy()
        self.assertEqual(len(frame_cells(mask,[],PROFILE,MOUNT,[0,0,0],0)),0)
        mask[:]=255;before=mask.copy()
        self.assertGreater(len(frame_cells(mask,[],PROFILE,MOUNT,[0,0,0],0)),0)
        self.assertEqual(len(frame_cells(mask,[[0,0,640,480]],PROFILE,MOUNT,[0,0,0],0)),0)
        np.testing.assert_array_equal(mask,before)

    def test_short_or_single_observation_does_not_publish_surface(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cv2.imwrite(str(root/'road.png'),np.full((480,640),255,np.uint8))
            frames=[dict(frame_id=str(i),camera_id='front',camera_time_s=i*.2,width=640,height=480) for i in range(3)]
            (root/'frames.json').write_text(json.dumps([dict(frame_id=str(i),road='road.png') for i in range(3)]))
            args=(frames,{},np.array([0.,.4]),np.zeros((2,3)),np.zeros(2),PROFILE,MOUNT)
            result=extract(root,*args)
            self.assertGreater(len(result['cells']),0)
            self.assertTrue(all(r['frames']==3 for r in result['cells']))
            result=extract(root,frames[:2],*args[1:])
            self.assertEqual(result['cells'],[])

    def test_strips_do_not_fill_unobserved_holes(self):
        result=rectangles([(0,0),(1,0),(3,0),(0,1)],1.)
        self.assertEqual(result,[[0.,0.,2.,1.],[3.,0.,4.,1.],[0.,1.,1.,2.]])

    def test_large_interior_island_is_not_filled_by_surface_completion(self):
        keys=[(x,y) for x in range(20) for y in range(20) if not (6<=x<14 and 6<=y<14)]
        mesh=surface_mesh(keys,.5);vertices=np.asarray(mesh['vertices_W_xy']);point=np.array([5.,5.])
        def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
        for ids in mesh['triangles_indices']:
            a,b,c=vertices[ids]
            inside=cross(b-a,point-a)>=0 and cross(c-b,point-b)>=0 and cross(a-c,point-c)>=0
            self.assertFalse(inside)
