import unittest
import numpy as np
from ct_scene.pocket_lane import fit_pockets,stations
from ct_scene.road_geometry import lane_at,lane_bounds,pocket_width
from ct_scene.lane_state import timeline
from ct_scene.surrounding import actor_corridor_lane


def peak(y):return dict(y=y,frames=4,frame_ids=['a','b','c','d'],cameras=['front'],origins=['ground_paint'])


def evidence():
    rows=[dict(x=x,kind='yellow',peaks=[peak(5.2)]) for x in [0,10,20,100,110]]
    rows.append(dict(x=30,kind='yellow',peaks=[peak(8.4)]))
    for x in range(10,100,10):
        p=[peak(5.2),peak(1.7)]
        if x>=40:p.append(peak(8.4))
        rows.append(dict(x=x,kind='white',peaks=p))
    return rows


class PocketLaneTests(unittest.TestCase):
    def setUp(self):self.road=dict(boundaries_y=[5.2,1.7,-1.8,-5.3],lane_width_m_prior=3.5,heading_evidence={'slope':0})

    def test_empty_pocket_comes_from_geometry_not_vehicle_presence(self):
        fit=fit_pockets(evidence(),self.road)
        self.assertEqual(len(fit['pockets']),1);self.assertFalse(fit['occupancy_used'])
        p=fit['pockets'][0];self.assertEqual((p['start_x'],p['end_x']),(20,100))
        self.assertEqual(pocket_width([p],0),0)
        self.assertGreater(pocket_width([p],25),0)
        self.assertLess(pocket_width([p],25),pocket_width([p],50))
        self.assertGreaterEqual(pocket_width([p],50),3.2)
        self.assertLessEqual(pocket_width([p],50),3.5)
        self.assertLess(pocket_width([p],95),pocket_width([p],50))
        self.assertEqual(pocket_width([p],110),0)

    def test_outward_curve_without_continuing_inner_separator_is_rejected(self):
        rows=evidence()
        for s in rows:
            if s['kind']=='white' and s['x']>=20:
                for p in s['peaks']:p['y']+=3
        self.assertFalse(fit_pockets(rows,self.road)['pockets'])

    def test_no_return_or_single_edge_cannot_establish_a_pocket(self):
        rows=[s for s in evidence() if s['kind']!='white']
        self.assertFalse(fit_pockets(rows,self.road)['pockets'])
        rows=[s for s in evidence() if not(s['kind']=='yellow' and s['x']>=100)]
        self.assertFalse(fit_pockets(rows,self.road)['pockets'])

    def test_main_ids_stay_fixed_and_opposite_side_is_outside(self):
        self.road['left_pockets']=fit_pockets(evidence(),self.road)['pockets']
        self.assertEqual(lane_at(self.road,50,6.5),-1)
        self.assertEqual(lane_at(self.road,10,3.5),0)
        self.assertEqual(lane_at(self.road,50,3.5),0)
        self.assertIsNone(lane_at(self.road,110,6.5))
        self.assertIsNone(lane_at(self.road,50,9))
        t=np.arange(0,13);p=np.c_[t*10,np.zeros(len(t)),np.zeros(len(t))]
        state=timeline(t,p,np.zeros(len(t)),self.road,4.7,2.1)
        self.assertTrue(all(r['lane_id']==1 for r in state['records']));self.assertFalse(state['events'])

    def test_pocket_vehicle_admission_rejects_oncoming_motion(self):
        self.road['left_pockets']=fit_pockets(evidence(),self.road)['pockets']
        a=dict(camera_id='front',samples=[dict(time_s=i*.2,position_W=[40+i,6.5,0],bbox_xyxy=[100,100,300,300]) for i in range(20)])
        self.assertEqual(actor_corridor_lane(a,self.road['boundaries_y'],0,self.road)[0],-1)
        for i,r in enumerate(a['samples']):r['position_W'][0]=70-i
        self.assertIsNone(actor_corridor_lane(a,self.road['boundaries_y'],0,self.road)[0])

    def test_station_support_counts_frames_not_duplicate_pixels(self):
        points=[dict(x=10,y=8,kind='yellow',frame_id='same',camera_id='front') for _ in range(500)]
        self.assertEqual(stations(points),[])


if __name__=='__main__':unittest.main()
