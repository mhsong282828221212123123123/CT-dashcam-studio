"""SEI lamp proxies, raw signal versus illustrative flash phase kept separate."""
import bisect,json,math
from pathlib import Path


def role(node,material):
    if material=='BrakeLightBrightLED':return 'brake'
    if material=='ReverseLightWhiteLED':return 'reverse'
    if material=='TurnSignalAmberLED':
        return 'left' if node in ('TurnSignal_FL','TurnSignal_SideL','TurnSignal_RL') else 'right' if node in ('TurnSignal_FR','TurnSignal_SideR','TurnSignal_RR') else None
    return None


class LampTrace:
    def __init__(self,rows,max_age_s=.2,flash_hz=1.5):
        self.rows=sorted((r for r in rows if r.get('status')=='decoded_v1'),key=lambda r:r['camera_time_s'])
        self.times=[r['camera_time_s'] for r in self.rows];self.max_age=max_age_s;self.flash_hz=flash_hz
        self.starts={'left':[],'right':[]};last={'left':None,'right':None};starts={'left':None,'right':None}
        for i,r in enumerate(self.rows):
            for side,key in [('left','blinker_on_left'),('right','blinker_on_right')]:
                f=r['fields'].get(key,{});value=f.get('value') if f.get('valid') else None
                gap=i>0 and r['camera_time_s']-self.times[i-1]>max_age_s
                if value is True and (last[side] is not True or gap):starts[side]=r['camera_time_s']
                if value is not True:starts[side]=None
                self.starts[side].append(starts[side]);last[side]=value

    def at(self,t):
        if not math.isfinite(t):raise ValueError('Finite lamp timestamp required')
        t=float(t)
        i=bisect.bisect_right(self.times,t)-1
        empty=dict(brake=None,left=None,right=None,reverse=None,left_lit=False,right_lit=False,source='unknown',flash_phase='illustrative_not_measured')
        if i<0 or t-self.times[i]>self.max_age:return empty
        fields=self.rows[i]['fields']
        def value(key):
            field=fields.get(key,{})
            return field.get('value') if field.get('valid') else None
        left,right=value('blinker_on_left'),value('blinker_on_right');gear=value('gear_state')
        def lit(side,active):
            start=self.starts[side][i]
            return bool(active is True and start is not None and (t-start)*self.flash_hz%1<.5)
        return dict(brake=value('brake_applied'),left=left,right=right,reverse=None if gear is None else gear==2,
                    left_lit=lit('left',left),right_lit=lit('right',right),source='sei_control_signals_not_optically_verified_bulbs',
                    signal_sample_time_s=self.times[i],flash_phase='illustrative_not_measured')

    @classmethod
    def from_artifact(cls,path):
        return cls([json.loads(l) for l in (Path(path)/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()])
