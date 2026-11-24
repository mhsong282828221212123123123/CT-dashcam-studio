"""Right-handed W/B: forward,left,up. Camera: right,down,forward."""
from dataclasses import dataclass
import math
import numpy as np


def validate_transform(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('Expected finite 4x4 transform')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError('Invalid homogeneous row')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-8, rtol=0) or not np.isclose(np.linalg.det(R), 1):
        raise ValueError('Rotation must be orthonormal and right-handed')
    return T


def transform(T_A_B, points_B):
    T = validate_transform(T_A_B)
    p = np.asarray(points_B, dtype=float)
    if p.shape[-1] != 3 or not np.isfinite(p).all():
        raise ValueError('Expected finite xyz points')
    return p @ T[:3, :3].T + T[:3, 3]


def inverse(T):
    T = validate_transform(T)
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return result


def front_camera_to_body():
    """Axis convention only; NOT a measured camera extrinsic."""
    T = np.eye(4)
    T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    return T


def project_pinhole(points_C, K, near=0.1):
    p = np.asarray(points_C, dtype=float)
    K = np.asarray(K, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or K.shape != (3, 3):
        raise ValueError('Invalid projection shapes')
    if not np.isfinite(p).all() or not np.isfinite(K).all() or near <= 0:
        raise ValueError('Invalid projection values')
    valid = p[:, 2] >= near
    pixels = np.full((len(p), 2), np.nan)
    q = p[valid] @ K.T
    pixels[valid] = q[:, :2] / q[:, 2:3]
    return pixels, valid


@dataclass(frozen=True)
class Clock:
    offset_s: float = 0.0
    scale: float = 1.0
    camera_epoch_s: float = 0.0
    session_epoch_s: float = 0.0
    status: str = 'unverified_prior'
    valid_from_camera_s: float | None = None
    valid_to_camera_s: float | None = None
    uncertainty_s: float | None = None

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.offset_s, self.scale, self.camera_epoch_s, self.session_epoch_s)) or self.scale <= 0:
            raise ValueError('Invalid clock')
        if self.status not in {'unverified_prior','estimated','verified'}:
            raise ValueError('Invalid clock status')
        endpoints=(self.valid_from_camera_s,self.valid_to_camera_s)
        if any(v is not None and not math.isfinite(v) for v in endpoints):
            raise ValueError('Invalid clock interval')
        if all(v is not None for v in endpoints) and endpoints[0]>endpoints[1]:
            raise ValueError('Reversed clock interval')
        if self.uncertainty_s is not None and (not math.isfinite(self.uncertainty_s) or self.uncertainty_s<0):
            raise ValueError('Invalid clock uncertainty')
        if self.status!='unverified_prior' and (None in endpoints or self.uncertainty_s is None):
            raise ValueError('Estimated/verified clocks require validity and uncertainty')

    def to_session(self, camera_time_s):
        if not math.isfinite(camera_time_s):raise ValueError('Invalid time')
        if self.valid_from_camera_s is not None and camera_time_s<self.valid_from_camera_s:
            raise ValueError('Clock before valid interval')
        if self.valid_to_camera_s is not None and camera_time_s>self.valid_to_camera_s:
            raise ValueError('Clock after valid interval')
        return self.session_epoch_s + self.scale * (camera_time_s-self.camera_epoch_s) + self.offset_s
