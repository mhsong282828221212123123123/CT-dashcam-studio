import copy,unittest
from ct_scene.input_preflight import check_coverage


class PreflightTests(unittest.TestCase):
    def cameras(self):
        return {c:dict(first_pts_s=.1,end_pts_s=60.1,max_gap_s=.04,missing_pts=0)
                for c in ('front','back','left_repeater','right_repeater')}

    def telemetry(self):
        return [dict(status='decoded_v1',camera_time_s=.1+i*.1,
                     fields={k:dict(valid=True) for k in ('vehicle_speed_mps','heading_deg')}) for i in range(600)]

    def test_full_coverage_and_b_frame_packet_order_are_supported(self):
        rows=self.telemetry();rows[1],rows[2]=rows[2],rows[1]
        self.assertEqual(check_coverage(self.cameras(),rows)['gate'],'PASS')

    def test_missing_tail_is_rejected_without_truncating_request(self):
        r=check_coverage(self.cameras(),self.telemetry()[:350])
        self.assertEqual(r['gate'],'FAIL')
        self.assertIn('telemetry_does_not_cover_front',[f['reason'] for f in r['failures']])
        self.assertEqual(r['front_interval_s'],[.1,60.1])

    def test_camera_short_tail_and_internal_gap_are_rejected(self):
        cams=self.cameras();cams['right_repeater']['end_pts_s']=45
        cams['left_repeater']['max_gap_s']=2
        failures=check_coverage(cams,self.telemetry())['failures']
        self.assertEqual({f['reason'] for f in failures},{'camera_does_not_cover_front','camera_timestamp_gap'})

    def test_invalid_or_missing_metadata_does_not_count_as_coverage(self):
        rows=self.telemetry()
        for row in rows:row['fields']['heading_deg']['valid']=False
        self.assertEqual(check_coverage(self.cameras(),rows)['failures'][0]['reason'],'no_usable_driving_telemetry')
        rows=self.telemetry();rows=rows[:200]+rows[220:]
        self.assertIn('telemetry_timestamp_gap',[f['reason'] for f in check_coverage(self.cameras(),rows)['failures']])

    def test_missing_camera_is_rejected(self):
        cams=self.cameras();del cams['front']
        self.assertEqual(check_coverage(cams,self.telemetry())['failures'][0]['reason'],'missing_camera_channels')
