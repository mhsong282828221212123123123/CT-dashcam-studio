import unittest
from ct_scene.source_strip import status
from ct_scene.input_preflight import check_coverage


class SourceStripTests(unittest.TestCase):
    def test_known_ended_camera_is_not_available(self):
        c=dict(back=dict(first_pts_s=0.,end_pts_s=59.15))
        self.assertEqual(status(c,'back',59.1),'available');self.assertEqual(status(c,'back',59.3),'recording_ended')
        self.assertEqual(status(c,'back',-.1),'recording_not_started')
    def test_legacy_or_unreliable_coverage_stays_unknown(self):
        self.assertEqual(status({},'back',60.),'unknown')
        self.assertEqual(status(dict(back=dict(first_pts_s=0,end_pts_s=None)),'back',60.),'unknown')
    def test_short_tail_is_reported_without_truncating_front_or_relaxing_gaps(self):
        cams={c:dict(first_pts_s=0.,end_pts_s=60.,max_gap_s=.04) for c in ['front','back','left_repeater','right_repeater']}
        rows=[dict(status='decoded_v1',camera_time_s=i*.1,fields={k:dict(valid=True) for k in ['vehicle_speed_mps','heading_deg']}) for i in range(600)]
        cams['back']['end_pts_s']=59.3;r=check_coverage(cams,rows)
        self.assertEqual(r['gate'],'PASS');self.assertEqual(r['front_interval_s'],[0.,60.]);self.assertEqual(r['warnings'][0]['camera'],'back')
        cams['back']['end_pts_s']=58.9;self.assertEqual(check_coverage(cams,rows)['gate'],'FAIL')
        cams['back']['end_pts_s']=59.3;cams['right_repeater']['max_gap_s']=1.5
        self.assertEqual(check_coverage(cams,rows)['gate'],'FAIL')
    def test_missing_start_and_telemetry_remain_failures(self):
        cams={c:dict(first_pts_s=0.,end_pts_s=60.,max_gap_s=.04) for c in ['front','back','left_repeater','right_repeater']}
        rows=[dict(status='decoded_v1',camera_time_s=i*.1,fields={k:dict(valid=True) for k in ['vehicle_speed_mps','heading_deg']}) for i in range(590)]
        self.assertEqual(check_coverage(cams,rows)['gate'],'FAIL')
        cams['left_repeater']['first_pts_s']=.6;self.assertIn('camera_does_not_cover_front',[r['reason'] for r in check_coverage(cams,rows)['failures']])
