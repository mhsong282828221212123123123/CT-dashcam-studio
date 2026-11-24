import copy,unittest
from ct_scene.scene_replay import validate_states,render_state


def state(t=0):
    return dict(time_s=t,ego_position_W=[0,0,0],ego_yaw_rad=0,
                camera_view=dict(weight=0,position_W=[0,0,0],yaw_rad=0),
                ego_lamps={'left':False},actors=[dict(id='car',model='sedan',position_W=[10,3,0],yaw_rad=.1,ghost=False)])


class ReplayTests(unittest.TestCase):
    def test_renderer_cannot_mutate_published_actor_or_lamp_state(self):
        class MutatingRenderer:
            def frame_many(self,actors,ego,yaw,lamps=None,**view):
                actors[0]['position_W'][0]=900
                actors[0]['id']='changed';lamps['left']=True
                return 'image'
        s=state();before=copy.deepcopy(s)
        self.assertEqual(render_state(MutatingRenderer(),s),'image')
        self.assertEqual(s,before)

    def test_missing_duplicate_and_nonfinite_states_rejected(self):
        with self.assertRaises(ValueError):validate_states([],{'frames':0})
        with self.assertRaises(ValueError):validate_states([state()],{'frames':2})
        s=state();s['actors']*=2
        with self.assertRaises(ValueError):validate_states([s],{'frames':1})
        s=state();s['ego_position_W'][0]=float('nan')
        with self.assertRaises(ValueError):validate_states([s],{'frames':1})

    def test_replay_preserves_clock_and_rejects_missing_frame(self):
        states=[state(0),state(.05),state(.1)]
        self.assertIs(validate_states(states,{'frames':3}),states)
        with self.assertRaises(ValueError):validate_states([state(0),state(.1)],{'frames':2})
