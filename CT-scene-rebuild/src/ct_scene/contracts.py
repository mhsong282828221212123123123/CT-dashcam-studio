"""Boundary records: invalid or unsynchronized values cannot appear measured."""
from dataclasses import dataclass
import math
from typing import Optional
from .ingest import CAMERAS


@dataclass(frozen=True)
class FrameRecord:
    source_sha256: str
    camera_id: str
    decode_index: int
    pts: int
    time_base_num: int
    time_base_den: int
    session_time_s: Optional[float] = None
    clock_status: str = 'unverified'

    def __post_init__(self):
        if len(self.source_sha256)!=64 or any(c not in '0123456789abcdef' for c in self.source_sha256):
            raise ValueError('Invalid source SHA-256')
        if self.camera_id not in CAMERAS:
            raise ValueError('Unknown camera')
        for name in ['decode_index','pts','time_base_num','time_base_den']:
            if type(getattr(self,name)) is not int:
                raise ValueError('Integer required: '+name)
        if self.decode_index<0 or self.time_base_num<=0 or self.time_base_den<=0:
            raise ValueError('Invalid frame index/time base')
        if self.clock_status not in {'unverified','estimated','verified'}:
            raise ValueError('Invalid clock status')
        if self.session_time_s is not None and (self.clock_status=='unverified' or not math.isfinite(self.session_time_s)):
            raise ValueError('Unverified/invalid session timestamp')

    @property
    def camera_time_s(self):
        return self.pts*self.time_base_num/self.time_base_den


@dataclass(frozen=True)
class ActorEvidenceState:
    image_presence: str
    position_support: str
    render_state: str
    metric_status: str

    def __post_init__(self):
        choices={
            'image_presence':{'visible','occluded','out_of_fov','unknown'},
            'position_support':{'multi_view','single_view','temporally_reconstructed','interpolated','predicted','unresolved'},
            'render_state':{'solid','ghost','unlocalized','hidden_scope','expired'},
            'metric_status':{'independently_validated','estimated_scale','unestablished'},
        }
        for name,allowed in choices.items():
            if getattr(self,name) not in allowed:raise ValueError('Invalid state: '+name)
        if self.position_support=='unresolved' and self.render_state in {'solid','ghost'}:
            raise ValueError('Unresolved position cannot become a positioned model')
        if self.render_state=='solid' and (self.image_presence!='visible' or self.position_support in {'interpolated','predicted'}):
            raise ValueError('Solid model requires current image support')
