import unittest
import numpy as np
import cv2
from ct_scene.manual_rig_probe import project
from ct_scene.plane_pose_probe import plane_points,plane_solutions,select_normal,fit_target_focal


class PlanePoseProbeTests(unittest.TestCase):
    profile={'model':'SIMPLE_RADIAL','params':[1000,640,480,-.1],'resolution':[1280,960]}

    def test_known_plane_normal_survives_homography_decomposition(self):
        a=np.array([(x,y) for y in (280,430,600) for x in (400,550,730,850)],float)
        normal=np.array([.15,.06,1.]);normal/=np.linalg.norm(normal)
        xyz=plane_points(a,normal,self.profile)
        R=cv2.Rodrigues(np.array([.03,.12,.02]))[0];pose=np.column_stack([R,[.12,.02,.04]])
        b=project(xyz,pose,self.profile)
        solutions=plane_solutions(a,b,self.profile)
        self.assertTrue(solutions)
        self.assertLess(min(np.linalg.norm(s['normal']-normal) for s in solutions),.005)

    def test_ambiguous_planes_cannot_be_chosen_by_target_error(self):
        candidates=[{'view':v,'normal':np.array(n,float)} for v in ('A','B','C') for n in ([0,0,1],[1,0,0])]
        with self.assertRaises(ValueError):select_normal(candidates)

    def test_focal_uses_oblique_fit_controls_only(self):
        xyz=np.array([[-.3,-.2,1],[.3,-.2,1],[-.3,.2,1],[.3,.2,1],[-.1,.05,1],[.2,.1,1]],float)
        actual=dict(self.profile,params=[900,*self.profile['params'][1:]])
        R=cv2.Rodrigues(np.array([.1,.6,.03]))[0];pose=np.column_stack([R,[.1,.02,.3]])
        pixels=project(xyz,pose,actual)
        candidate,diagnostic=fit_target_focal(xyz,pixels,self.profile)
        self.assertLess(abs(candidate['params'][0]-900),2)
        self.assertFalse(diagnostic['check_pixels_used'])
