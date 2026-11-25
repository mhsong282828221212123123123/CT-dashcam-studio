"""Step 2 gated side-camera ground-contact & height-prior uncertainty fusion.

Replaces unconditional ``apparent_vehicle_height_prior`` on ``left_repeater`` /
``right_repeater`` with inverse-variance fusion of gated ground-contact rays
(preferably from ``contact-adapter-v7``) and apparent-height depth priors, plus
within-track height-scale calibration so ungated tail frames never jump.
"""
import math
import numpy as np


def fuse_side_sample(row, frame_w, frame_h, profile, mount, vehicle_dims,
                     project_contact_fn, project_height_fn, contact_uv=None, camera='left_repeater'):
    """Return fused ego-relative 3D position and explicit uncertainty metadata."""
    box = row['bbox_xyxy']
    w = float(box[2] - box[0])
    h = float(box[3] - box[1])
    veh_h = float(vehicle_dims[2])
    rel_h = np.asarray(project_height_fn(box, profile, mount, veh_h), float)
    mount_off = np.asarray(mount['offset_W_ego_m'], float)
    vec_h = rel_h[:2] - mount_off[:2]
    rh = float(np.linalg.norm(vec_h))
    sigma_h = 1.25 + 0.28 * rh

    margin = float(min(box[0], box[1], frame_w - box[2], frame_h - box[3]))
    aspect = w / max(h, 1.0)
    gate_basic = (
        row.get('support') == 'detector_observation'
        and 'truncated' not in row.get('quality_flags', [])
        and w >= 34.0
        and h >= 28.0
        and 0.55 <= aspect <= 3.5
        and margin >= 5.0
        and float(box[3]) >= frame_h * 0.48
    )
    if not gate_basic:
        quality = float(np.clip(1.0 / (1.0 + 0.8 * sigma_h * sigma_h), 0.15, 0.35))
        return dict(
            relative_ego_m=rel_h,
            relative_height_m=rel_h,
            projection_support='apparent_vehicle_height_prior',
            position_sigma_m=float(sigma_h),
            contact_fusion_weight=0.0,
            quality=quality,
            raw_contact_ego_m=None,
        )

    if (contact_uv is not None and len(contact_uv) == 2
            and np.isfinite(contact_uv).all()
            and 0.06 <= float(contact_uv[0]) <= 0.94
            and 0.75 <= float(contact_uv[1]) <= 1.08):
        uv = [float(contact_uv[0]), float(contact_uv[1])]
        uv_source = 'learned_contact'
    else:
        uv = [0.5, 0.985]
        uv_source = 'bbox_bottom_fallback'

    try:
        rel_c = np.asarray(
            project_contact_fn(box, profile, mount, max_depth=36.0, minimum_ray_y=0.022, contact_uv=uv),
            float,
        )
    except ValueError:
        quality = float(np.clip(1.0 / (1.0 + 0.8 * sigma_h * sigma_h), 0.15, 0.35))
        return dict(
            relative_ego_m=rel_h,
            relative_height_m=rel_h,
            projection_support='apparent_vehicle_height_prior',
            position_sigma_m=float(sigma_h),
            contact_fusion_weight=0.0,
            quality=quality,
            raw_contact_ego_m=None,
        )

    vec_c = rel_c[:2] - mount_off[:2]
    rc = float(np.linalg.norm(vec_c))
    # Reject raised-curb / elevated-background boxes whose ground-plane ray
    # shoots far beyond apparent vehicle-size range.
    if rc > 34.0 or rc > 1.25 * rh + 4.0 or rc < .55 * rh - 1.0:
        quality = float(np.clip(1.0 / (1.0 + 0.8 * sigma_h * sigma_h), 0.15, 0.35))
        return dict(
            relative_ego_m=rel_h,
            relative_height_m=rel_h,
            projection_support='apparent_vehicle_height_prior',
            position_sigma_m=float(sigma_h),
            contact_fusion_weight=0.0,
            quality=quality,
            raw_contact_ego_m=None,
        )

    size_ramp = float(np.clip((w - 30.0) / 25.0, 0.2, 1.0) * np.clip((h - 24.0) / 20.0, 0.2, 1.0))
    margin_ramp = float(np.clip((margin - 4.0) / 10.0, 0.25, 1.0))
    range_ramp = float(np.clip((34.5 - rc) / 8.0, 0.0, 1.0) * np.clip((1.25 * rh + 4.0 - rc) / 4.0, 0.0, 1.0))
    alpha = size_ramp * margin_ramp * range_ramp
    if alpha <= 1e-3:
        quality = float(np.clip(1.0 / (1.0 + 0.8 * sigma_h * sigma_h), 0.15, 0.35))
        return dict(
            relative_ego_m=rel_h,
            relative_height_m=rel_h,
            projection_support='apparent_vehicle_height_prior',
            position_sigma_m=float(sigma_h),
            contact_fusion_weight=0.0,
            quality=quality,
            raw_contact_ego_m=rel_c,
        )

    # Shift from visible near-side tire line toward 3D vehicle body centre when using learned tire contact.
    rel_c_center = rel_c.copy()
    if uv_source == 'learned_contact':
        lat_sign = 1.0 if camera == 'left_repeater' else -1.0
        sin_view = abs(float(vec_c[1])) / max(rc, 1.0)
        half_shift = min(0.42, float(vehicle_dims[1]) * 0.22) * sin_view
        rel_c_center[1] += lat_sign * half_shift

    sigma_pixel = (0.20 if uv_source == 'learned_contact' else 0.35) + 0.0032 * (rc * rc)
    sigma_pitch = rc*rc/max(float(mount['height_m']),.1)*float(mount.get('pitch_sigma_rad',.025))
    # Class-height uncertainty is systematic; gross disagreement must not gain
    # 99% weight merely because the ground ray is artificially near the camera.
    sigma_disagreement=max(0.,abs(rc-rh)-.18*rh)
    sigma_c = math.sqrt(sigma_pixel**2+sigma_pitch**2+sigma_disagreement**2)
    prec_c = alpha / (sigma_c * sigma_c)
    prec_h = 1.0 / (sigma_h * sigma_h)
    w_c = float(prec_c / (prec_c + prec_h))
    rel_fused = w_c * rel_c_center + (1.0 - w_c) * rel_h
    sigma_yaw=float(np.linalg.norm(rel_fused[:2]-mount_off[:2]))*float(mount.get('yaw_sigma_rad',0.))
    sigma_fused = float(math.sqrt(1.0/(prec_c+prec_h)+sigma_yaw**2))
    quality = float(np.clip(1.0 / (1.0 + 0.8 * sigma_fused * sigma_fused), 0.15, 0.85))
    support = 'gated_side_ground_contact_fusion' if w_c >= 0.5 else 'apparent_vehicle_height_prior'
    return dict(
        relative_ego_m=rel_fused,
        relative_height_m=rel_h,
        projection_support=support,
        position_sigma_m=sigma_fused,
        contact_fusion_weight=w_c,
        quality=quality,
        raw_contact_ego_m=rel_c,
        diagnostics=dict(pitch_calibration_accepted=bool(mount.get('pitch_calibration_accepted')),yaw_sigma_rad=float(mount.get('yaw_sigma_rad',0.)),clear_ground_candidate=bool(gate_basic),height_range_m=rh,contact_range_m=rc,pitch_correction_rad=float(mount.get('pitch_correction_rad',0.)),pitch_sigma_m=sigma_pitch,yaw_sigma_m=sigma_yaw,disagreement_sigma_m=sigma_disagreement),
    )


def calibrate_side_height_tails(samples, ego_times, ego_positions, ego_yaws, mount, rotation_fn):
    """Align ungated distant/border height-prior tail samples to the track's gated contact scale."""
    gated_ratios = []
    mount_off = np.asarray(mount['offset_W_ego_m'], float)
    for s in samples:
        if s.get('contact_fusion_weight', 0.0) < 0.65:
            continue
        rel_f = s.get('_rel_fused_ego')
        rel_h = s.get('_rel_height_ego')
        if rel_f is None or rel_h is None:
            continue
        rf = float(np.linalg.norm(np.asarray(rel_f[:2], float) - mount_off[:2]))
        rh = float(np.linalg.norm(np.asarray(rel_h[:2], float) - mount_off[:2]))
        if rh > 2.5 and rf > 1.5:
            gated_ratios.append(rf / rh)
    scale = float(np.median(gated_ratios)) if len(gated_ratios)>=5 else None
    if not mount.get('pitch_calibration_accepted') or scale is None or not .78<=scale<=1.25:
        scale=None

    for s in samples:
        rel_f = s.pop('_rel_fused_ego', None)
        rel_h = s.pop('_rel_height_ego', None)
        is_trunc = s.pop('_is_truncated', False)
        if scale is None or rel_f is None or rel_h is None or is_trunc:
            continue
        wc = float(s.get('contact_fusion_weight', 0.0))
        if wc >= 0.5:
            continue
        rel_h_arr = np.asarray(rel_h, float)
        rh = float(np.linalg.norm(rel_h_arr[:2] - mount_off[:2]))
        if rh < 10.0:
            continue
        t = float(s['time_s'])
        yaw = float(np.interp(t, ego_times, ego_yaws))
        ep = np.array([np.interp(t, ego_times, ego_positions[:, k]) for k in range(3)], float)
        rel_cal = mount_off + scale * (rel_h_arr - mount_off)
        rel_cal[2] = 0.0
        rel_blend = wc * np.asarray(rel_f, float) + (1.0 - wc) * rel_cal
        pw = ep + rotation_fn(yaw) @ rel_blend
        s['position_W'] = pw.tolist()
        s['unrefined_position_W'] = pw.tolist()
    return scale
