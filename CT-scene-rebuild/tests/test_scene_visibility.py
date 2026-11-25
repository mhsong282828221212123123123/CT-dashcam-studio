import unittest
from ct_scene.scene_visibility import build,visible_at


class SourceFrameVisibilityTests(unittest.TestCase):
    def test_successive_camera_frames_do_not_make_two_ids_simultaneously_observed(self):
        fs=[dict(frame_id='f1',camera_id='right_repeater',camera_time_s=50.3944),dict(frame_id='f2',camera_id='right_repeater',camera_time_s=50.586)]
        obs=[dict(fs[0],local_track_id='old',support='detector_observation'),dict(fs[1],local_track_id='new',support='detector_observation'),dict(fs[1],local_track_id='old',support='offline_two_endpoint_gap')]
        idx=build(fs,obs,dict(old='a',new='b'))
        self.assertEqual(visible_at(idx,50.5),{'b'})
        self.assertEqual(visible_at(idx,50.4),{'a'})
    def test_missing_detections_do_not_inherit_visible_id_from_previous_frame(self):
        fs=[dict(frame_id=str(i),camera_id='front',camera_time_s=i*.2) for i in range(3)]
        idx=build(fs,[dict(fs[0],local_track_id='a',support='detector_observation')],dict(a='a'))
        self.assertEqual(visible_at(idx,.19),set())
        self.assertEqual(visible_at(idx,1.),set())
    def test_camera_union_and_rear_identity_reference(self):
        fs=[dict(frame_id='f',camera_id='front',camera_time_s=1.),dict(frame_id='b',camera_id='back',camera_time_s=1.)]
        obs=[dict(fs[0],local_track_id='f',support='detector_observation'),dict(fs[1],local_track_id='b',support='flow_observation')]
        idx=build(fs,obs,dict(f='front_car'),dict(intervals=[dict(rear_id='b',global_id='rear_car',start_s=.5,end_s=1.5)]))
        self.assertEqual(visible_at(idx,1.),{'front_car','rear_car'})
