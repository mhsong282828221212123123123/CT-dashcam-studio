import copy
import unittest
from ct_scene.manual_review import validate,apply


class ManualReviewTests(unittest.TestCase):
    def setUp(self):
        self.manifest={'artifact_id':'original'}
        self.states=[dict(time_s=t,actors=[dict(id='car',position_W=[10.,2.,0.],yaw_rad=0.,position_support='observed_geometry')],camera_view={'unchanged':True}) for t in [0.,1.,2.]]
        self.layer=dict(schema='ct_scene.manual_corrections.v1',source_artifact_id='original',edits=[dict(actor_id='car',reason='raw source review',keyframes=[dict(time_s=t,offset_W_xy_m=xy,yaw_delta_deg=a) for t,xy,a in [(0.,[0.,0.],0.),(1.,[2.,-1.],30.),(2.,[0.,0.],0.)]])])

    def test_interpolation_no_automatic_mutation(self):
        original=copy.deepcopy(self.states);validate(self.layer,self.manifest,self.states)
        half=apply(self.layer,dict(self.states[1],time_s=.5))
        self.assertEqual(half['actors'][0]['position_W'],[11.,1.5,0.])
        self.assertAlmostEqual(half['actors'][0]['yaw_rad'],.2617993877991494)
        self.assertEqual(self.states,original);self.assertEqual(half['camera_view'],original[1]['camera_view'])
        self.assertEqual(half['actors'][0]['automatic_position_W'],[10.,2.,0.])
        self.assertEqual(apply(self.layer,self.states[0]),self.states[0])
        self.assertEqual(apply(self.layer,self.states[-1]),self.states[-1])

    def test_wrong_result_unknown_actor_or_ego_rejected(self):
        for field,value in [('source_artifact_id','wrong')]:
            layer=copy.deepcopy(self.layer);layer[field]=value
            with self.assertRaises(ValueError):validate(layer,self.manifest,self.states)
        for key in ['ego','missing']:
            layer=copy.deepcopy(self.layer);layer['edits'][0]['actor_id']=key
            with self.assertRaises(ValueError):validate(layer,self.manifest,self.states)

    def test_discontinuity_overlap_nan_rejected(self):
        for mutation in ['endpoint','overlap','nan','order']:
            layer=copy.deepcopy(self.layer);edit=layer['edits'][0]
            if mutation=='endpoint':edit['keyframes'][0]['offset_W_xy_m'][0]=1
            if mutation=='overlap':layer['edits'].append(copy.deepcopy(edit))
            if mutation=='nan':edit['keyframes'][1]['yaw_delta_deg']=float('nan')
            if mutation=='order':edit['keyframes'][1]['time_s']=2.
            with self.assertRaises(ValueError):validate(layer,self.manifest,self.states)

    def test_no_invention_of_unobserved_actor(self):
        validate(self.layer,self.manifest,self.states)
        state=dict(time_s=1.,actors=[])
        self.assertEqual(apply(self.layer,state),state)
