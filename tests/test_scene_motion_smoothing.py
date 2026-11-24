import math,unittest
import numpy as np
from core.driving_scene import Actor,Scene
from core.scene_world_tracks import WorldTracks
from core.scene_fusion import CameraGeometry,corners,dimensions
from core.scene_motion_smoothing import smooth_motion


class MotionSmoothingTests(unittest.TestCase):
    def fixture(self):
        tracker=WorldTracks();camera=CameraGeometry(horizon=500)
        scenes=[];poses=[];states=[];rows=[];audits=[]
        for i in range(40):
            t=i*.1;x=-6+4*t;noisy=x+.4*math.sin(i*2)
            measured=Actor('car','car',noisy,30,yaw=math.pi/2,sources=('front:1',))
            actors,_=tracker.update([measured],t,{'speed_kmh':0})
            scenes.append(Scene(t,{'speed_kmh':0},actors=actors));poses.append((np.zeros(2),np.eye(2)))
            states.append({a.key:(tracker.tracks[a.key]['state'].copy(),tracker.tracks[a.key]['cov'].copy()) for a in actors})
            p=camera.visible_bounds(corners(x,30,dimensions('car'),yaw=math.pi/2,kind='car'))
            rows.append(dict(cameras=dict(front=dict(objects=[dict(track_id='front:1',bbox=np.r_[p[:2],p[2:]-p[:2]].tolist())]))));audits.append({})
        return scenes,poses,states,rows,[{'front':camera}]*40,audits

    def test_future_measurements_reduce_jitter_without_changing_identity(self):
        args=self.fixture();scenes=args[0];before=np.array([s.actors[0].x for s in scenes])
        count=smooth_motion(*args,eligible=set(range(40)))
        after=np.array([s.actors[0].x for s in scenes])
        self.assertEqual(count,40)
        self.assertLess(np.mean(np.diff(after,n=2)**2),np.mean(np.diff(before,n=2)**2))
        self.assertTrue(all(s.actors[0].key=='car' for s in scenes))

    def test_invalid_frame_is_retained_without_disabling_other_valid_runs(self):
        args=self.fixture();before=[(s.actors[0].x,s.actors[0].z) for s in args[0]]
        args[3][20]['cameras']['front']['objects'][0]['bbox']=[0,0,10,10]
        count=smooth_motion(*args,eligible=set(range(40)))
        self.assertGreater(count,0);self.assertLess(count,40)
        self.assertEqual(before[20],(args[0][20].actors[0].x,args[0][20].actors[0].z))
