import unittest
import numpy as np
from ct_scene.image_median_gate import review


class PixelMedianTests(unittest.TestCase):
    def test_clipped_exit_width_does_not_erase_clear_oncoming_approach(self):
        t,rows,lines=self.fixture()
        for i,r in enumerate(rows):
            r['bbox_xyxy']=[180-i*22,450,290+i*6,570+i*12] if i<6 else [0,450,210-(i-6)*65,570+i*12]
        for line in lines:line['pixels']=[420,850,470,600]
        result=review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']
        self.assertTrue(result['excluded']);self.assertTrue(result['approaching_size_and_contact'])
    def test_approaching_oncoming_body_can_overlap_yellow_in_its_box(self):
        t,rows,lines=self.fixture()
        for i,r in enumerate(rows):
            # The near-corner bbox straddles the median even though the body
            # centre stays left. Size/contact grow as the oncoming car nears.
            r['bbox_xyxy']=[300-i*12,450,470+i,570+i*7]
        for line in lines:line['pixels']=[420,850,470,600]
        result=review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']
        self.assertTrue(result['excluded']);self.assertEqual(result['beyond_samples'],0)
        for r in rows[-3:]:r['bbox_xyxy']=[550,450,750,700]
        self.assertFalse(review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']['excluded'])

    def fixture(self):
        t=np.arange(0,2,.2);rows=[dict(time_s=float(s),frame_id=str(i),support='detector_observation',bbox_xyxy=[30,500,130,650],image_width=1280,image_height=960) for i,s in enumerate(t)]
        lines=[dict(kind='yellow',frame_id=str(i),pixels=[300,850,500,600]) for i in range(len(t))]
        return t,rows,lines

    def test_front_vehicle_wholly_beyond_median_can_be_ignored_without_xyz(self):
        t,rows,lines=self.fixture();r=review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)
        self.assertTrue(r['car']['excluded']);self.assertFalse(r['car']['metric_projection_used'])

    def test_crossing_side_view_and_future_own_side_are_preserved(self):
        t,rows,lines=self.fixture()
        for i,r in enumerate(rows):r['bbox_xyxy']=[10+i*20,450,210+i*20,520]
        self.assertFalse(review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']['excluded'])
        t,rows,lines=self.fixture()
        for r in rows[-3:]:r['bbox_xyxy']=[550,500,650,650]
        self.assertFalse(review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']['excluded'])

    def test_ego_turn_and_repeater_do_not_supply_exclusion(self):
        t,rows,lines=self.fixture()
        self.assertEqual(review([dict(id='car',camera_id='front',samples=rows)],lines,t,np.radians(t*6)),{})
        self.assertEqual(review([dict(id='car',camera_id='left_repeater',samples=rows)],lines,t,t*0),{})

    def test_approaching_oblique_oncoming_body_is_not_a_crossing_by_aspect_alone(self):
        t,rows,lines=self.fixture()
        for i,r in enumerate(rows):r['bbox_xyxy']=[10,500+i*5,150+i*2,560+i*6]
        for line in lines:line['pixels']=[400,850,450,600]
        self.assertTrue(review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']['excluded'])

    def test_retreating_side_view_beyond_median_abstains(self):
        t,rows,lines=self.fixture()
        for i,r in enumerate(rows):r['bbox_xyxy']=[10,450-i,210-i*2,520-i]
        r=review([dict(id='car',camera_id='front',samples=rows)],lines,t,t*0)['car']
        self.assertFalse(r['excluded']);self.assertTrue(r['crossing_guard'])
