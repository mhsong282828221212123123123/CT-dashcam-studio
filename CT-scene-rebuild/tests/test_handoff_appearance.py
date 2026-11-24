import copy
import unittest
import numpy as np
from ct_scene.handoff_appearance import review
from ct_scene.surrounding import association_edges


class HandoffAppearanceTests(unittest.TestCase):
    def views(self,key,values):
        return [dict(frame_id=key+str(i),time_s=i*.2,histogram=value) for i,value in enumerate(values)]

    def test_repeated_views_not_one_lucky_match_support_the_distance(self):
        a=dict(clear_appearance_views=self.views('A',[[1.,0.]]*6))
        b=dict(clear_appearance_views=self.views('B',[[0.,1.]]*5+[[1.,0.]]))
        before=copy.deepcopy((a,b));result=review(a,b)
        self.assertEqual(result['minimum'],0.);self.assertEqual(result['distance'],1.)
        self.assertEqual((a,b),before)
        b['clear_appearance_views']=self.views('B',[[1.,0.]]*6)
        self.assertEqual(review(a,b)['distance'],0.)

    def test_duplicate_or_brief_views_do_not_supply_independent_support(self):
        a=dict(clear_appearance_views=self.views('A',[[1.,0.]]*6))
        self.assertIsNone(review(a,dict(clear_appearance_views=[a['clear_appearance_views'][0]]*8)))
        b=copy.deepcopy(a)
        for r in b['clear_appearance_views']:r['time_s']*=.1
        self.assertIsNone(review(a,b))

    def pair(self):
        result=[]
        for key,camera,times,pending in [('A','front',np.arange(0,1.81,.2),False),('B','right_repeater',np.arange(2.2,4.01,.2),True)]:
            rows=[dict(time_s=float(t),frame_id=key+str(i),position_W=[10.+t,0.,0.],
                       bbox_xyxy=[100.,200.,400.,500.],image_width=1280,image_height=960,
                       support='detector_observation',yaw_rad=0.) for i,t in enumerate(times)]
            rows[-1 if camera=='front' else 0]['bbox_xyxy']=[1260.,200.,1280.,500.] if camera=='front' else [0.,200.,30.,500.]
            result.append(dict(id=key,camera_id=camera,start_s=rows[0]['time_s'],end_s=rows[-1]['time_s'],
                               samples=rows,lane_index=-2 if pending else 0,scope_pending=pending,
                               appearance=[1.,0.],identity_evidence=dict(usable_observations=8)))
        return result

    def test_unknown_scope_requires_independent_clear_motion_and_admitted_peer(self):
        a,b=self.pair();edges=association_edges([a,b]);self.assertEqual(len(edges),1)
        self.assertTrue(edges[0]['selected']);self.assertTrue(edges[0]['scope_deferred_until_identity'])
        a['scope_pending']=True;self.assertEqual(association_edges([a,b]),[])
        a,b=self.pair()
        for r in b['samples']:r['position_W'][1]=20.
        self.assertEqual(association_edges([a,b]),[])

    def test_clear_view_fallback_still_rejects_ambiguous_alternatives(self):
        a,b=self.pair();b['appearance']=[0.,1.]
        a['clear_appearance_views']=self.views('FA',[[1.,0.]]*6)
        b['clear_appearance_views']=self.views('SB',[[1.,0.]]*6)
        edges=association_edges([a,b]);self.assertTrue(edges[0]['selected'])
        self.assertEqual(edges[0]['original_appearance_distance'],1.)
        self.assertEqual(edges[0]['clear_view_appearance_review']['distance'],0.)
        other=copy.deepcopy(b);other['id']='C'
        edges=association_edges([a,b,other]);self.assertEqual(len(edges),2)
        self.assertTrue(all(not e['selected'] for e in edges))
