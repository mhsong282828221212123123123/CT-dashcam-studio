import unittest
import numpy as np
from ct_scene.steering_cue import SteeringCue
from ct_scene.ego_turn_rotation import corrected_yaw

class SteeringDirectionTests(unittest.TestCase):
    def fixture(self):
        t=np.arange(0,60,.05);raw=30*np.sin(t*.25)+2
        rate=.004*(raw-2)*10
        heading=180-np.degrees(np.cumsum(rate)*.05)
        return [dict(camera_time_s=float(x),fields={k:dict(value=float(v),valid=True,presence='wire') for k,v in [('steering_wheel_angle_raw',r),('heading_deg',h),('vehicle_speed_mps',10)]}) for x,r,h in zip(t,raw,heading)]

    def test_unknown_units_sign_and_zero_calibrate_against_own_motion(self):
        rows=self.fixture();cue=SteeringCue(rows)
        self.assertEqual(cue.report['status'],'validated_direction_reference')
        self.assertGreater(cue.rate_at(4),0)
        for r in rows:r['fields']['steering_wheel_angle_raw']['value']*=-20
        reversed_units=SteeringCue(rows)
        self.assertEqual(reversed_units.report['status'],'validated_direction_reference')
        self.assertAlmostEqual(cue.rate_at(4),reversed_units.rate_at(4),places=8)

    def test_default_or_constant_steering_abstains(self):
        rows=self.fixture()
        for r in rows:r['fields']['steering_wheel_angle_raw']['presence']='schema_default'
        self.assertIsNone(SteeringCue(rows).rate_at(4))
        rows=self.fixture()
        for r in rows:r['fields']['steering_wheel_angle_raw']['value']=0
        self.assertIsNone(SteeringCue(rows).rate_at(4))

    def test_opposite_visual_turn_is_vetoed_without_integrating_steering(self):
        t=np.arange(0,15.01,.2);yaw=-np.pi/2*np.clip((t-5)/5,0,1)
        # A short wrong-way visual interval compensated later keeps the endpoint;
        # the old endpoint gate alone therefore accepts it.
        increments=np.diff(yaw);wrong=(t[:-1]>=6)&(t[:-1]<6.4)
        increments[wrong]*=-1
        compensation=np.sum(np.diff(yaw))-sum(increments)
        increments[(t[:-1]>=7)&(t[:-1]<8)]+=compensation/5
        pairs={c:[dict(start_s=float(a),end_s=float(b),accepted=True,visual_delta_deg=float(np.degrees(d))) for a,b,d in zip(t[:-1],t[1:],increments)] for c in ['front','left_repeater']}
        class Cue:
            def rate_at(self,time):return -.3 if 6<=time<6.4 else None
        old,_=corrected_yaw(t,yaw,pairs)
        result,review=corrected_yaw(t,yaw,pairs,Cue())
        self.assertTrue(any(r['steering_direction_rejected_intervals'] for r in review))
        self.assertLess(np.max(np.diff(result)[wrong]),0)
        self.assertGreater(np.max(np.diff(old)[wrong]),0)
        self.assertAlmostEqual(result[-1],yaw[-1])
