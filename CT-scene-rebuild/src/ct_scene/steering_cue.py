"""Clip-calibrated steering direction cue; never a body-heading measurement.

Raw SEI units/sign are unspecified. Learn a signed curvature relation against
own SEI heading, validate on held-out seconds, and abstain without diversity.
No actor, GPS position, road assignment or steering-ratio assumption is used.
"""
import numpy as np
from .wheel_scale import unique_motion_rows


class SteeringCue:
    def __init__(self, rows):
        self.report = dict(status='unavailable', use='own_turn_direction_veto_only',
                           raw_unit='unverified', body_heading_from_steering=False)
        self.times = np.array([]); self.rates = np.array([])
        rows = [r for r in unique_motion_rows(rows) if all(
            r.get('fields', {}).get(k, {}).get('valid') and
            r['fields'][k].get('presence') == 'wire'
            for k in ('steering_wheel_angle_raw', 'heading_deg', 'vehicle_speed_mps'))]
        if len(rows) < 60: return
        t = np.array([r['camera_time_s'] for r in rows])
        raw = np.array([r['fields']['steering_wheel_angle_raw']['value'] for r in rows])
        speed = np.array([r['fields']['vehicle_speed_mps']['value'] for r in rows])
        heading = -np.unwrap(np.radians([r['fields']['heading_deg']['value'] for r in rows]))
        grid = np.arange(t[0]+.6, t[-1]-.6, .2)
        if len(grid) < 50: return
        # Centred one-second body rotation avoids derivative noise at frame rate.
        rate = (np.interp(grid+.5,t,heading)-np.interp(grid-.5,t,heading))
        wheel = np.array([np.median(raw[abs(t-x)<.3]) for x in grid])
        v = np.interp(grid,t,speed)
        nearest = np.searchsorted(t,grid).clip(1,len(t)-1)
        valid = (v>3) & ((t[nearest]-t[nearest-1])<.2)
        gaps=np.flatnonzero(np.diff(t)>.2)
        for i in gaps: valid &= ~((grid+.6>=t[i]) & (grid-.6<=t[i+1]))
        train = valid & ((np.floor(grid).astype(int)%2)==0)
        test = valid & ~train
        if train.sum()<25 or test.sum()<25: return
        y=rate/np.maximum(v,1); x=wheel
        if np.ptp(x[train])<5 or np.count_nonzero(abs(rate[train])>np.radians(3))<15:
            self.report['status']='insufficient_turn_diversity'; return
        fit=train.copy()
        for _ in range(3):
            gain,offset=np.polyfit(x[fit],y[fit],1)
            residual=y-(gain*x+offset)
            scale=max(1e-5,1.4826*np.median(abs(residual[fit]-np.median(residual[fit]))))
            fit=train & (abs(residual)<3*scale)
            if fit.sum()<25: return
        predicted=(gain*x+offset)*v
        turning=test & (abs(rate)>np.radians(3)) & (abs(predicted)>np.radians(3))
        if turning.sum()<15: return
        agreement=float(np.mean(np.sign(rate[turning])==np.sign(predicted[turning])))
        error=float(np.degrees(np.median(abs(rate[test]-predicted[test]))))
        self.report.update(held_out_direction_samples=int(turning.sum()),
            held_out_direction_agreement=agreement,held_out_median_rate_error_deg_s=error,
            signed_gain_per_raw_unit=float(gain),raw_zero=float(-offset/gain) if abs(gain)>1e-10 else None)
        if agreement<.95 or error>3 or abs(gain)<1e-7:
            self.report['status']='calibration_rejected'; return
        self.report['status']='validated_direction_reference'
        self.times=grid
        self.rates=np.where(valid,predicted,np.nan)

    def rate_at(self,time):
        if not len(self.times): return None
        window=self.rates[abs(self.times-time)<=.4]
        if len(window)<3 or not np.isfinite(window).all(): return None
        if not (np.all(window>np.radians(4)) or np.all(window<-np.radians(4))): return None
        return float(np.median(window))
