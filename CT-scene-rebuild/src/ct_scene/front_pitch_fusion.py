"""Step 3 per-frame front camera pitch compensation & far-range size-contact fusion.

1. Estimates bounded relative pitch fluctuations ``delta_pitch_rad(t)`` from
   unoccluded central road-mask horizon rays, preventing uphill/downhill grades
   and chassis pitch from exploding ``1 / ray_y`` at long range.
2. Fuses pitch-compensated ground-contact depth (error ~ z^2) with self-calibrated
   apparent vehicle-size depth (error ~ z) by inverse variance, suppressing
   50m+ jitter spikes and border/occlusion depth blowups while preserving sub-30cm
   near-field tire-contact accuracy.
"""
import json
import math
from pathlib import Path
import cv2
import numpy as np
from .affine_probe import camera_matrix


def estimate_front_pitch_profile(road_masks, front_frames, observations, profile):
    """Estimate smooth bounded relative pitch offsets (radians) per front frame."""
    if not road_masks or not front_frames:
        return dict(status='disabled_no_road_masks', by_frame_id={}, reference_ray_y=None)
    rm_path = Path(road_masks)
    frames_file = rm_path / 'frames.json'
    if not frames_file.is_file():
        return dict(status='disabled_missing_mask_index', by_frame_id={}, reference_ray_y=None)

    records = {r['frame_id']: r for r in json.loads(frames_file.read_text(encoding='utf-8'))}
    K, D = camera_matrix(profile)
    raw_rows = []
    for f in front_frames:
        fid = f['frame_id']
        rec = records.get(fid)
        if rec is None or 'road' not in rec:
            continue
        mask = cv2.imread(str(rm_path / rec['road']), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            continue
        h, w = mask.shape[:2]
        x_lo, x_hi = int(round(w * 0.34)), int(round(w * 0.66))
        col_blocked = np.zeros(x_hi - x_lo, bool)
        for obs in observations.get(fid, []):
            b = obs['bbox_xyxy']
            # Any vehicle near/crossing the horizon blocks true road termination in those columns
            if float(b[1]) <= h * 0.60 and float(b[3]) >= h * 0.44:
                c0 = max(0, int(math.floor(b[0] - 12)) - x_lo)
                c1 = min(x_hi - x_lo, int(math.ceil(b[2] + 12)) - x_lo)
                if c1 > c0:
                    col_blocked[c0:c1] = True
        unoccluded_cols = np.flatnonzero(~col_blocked)
        if len(unoccluded_cols) < 60:
            continue
        band = mask[:, x_lo:x_hi] > 0
        top_ys = []
        y_min_scan = int(round(h * 0.42))
        for col in unoccluded_cols:
            nz = np.flatnonzero(band[y_min_scan:, col])
            if len(nz) < 28:
                continue
            y0 = int(nz[0]) + y_min_scan
            if y0 + 16 <= h and np.count_nonzero(band[y0:y0 + 16, col]) >= 12:
                top_ys.append(y0)
        if len(top_ys) < 40:
            continue
        hy = float(np.percentile(top_ys, 20))
        ry = float(cv2.undistortPoints(np.array([[[w * 0.5, hy]]], float), K, D).reshape(2)[1])
        if 0.002 <= ry <= 0.060:
            raw_rows.append((float(f['camera_time_s']), fid, ry))

    if len(raw_rows) < 8:
        return dict(status='insufficient_unoccluded_horizon_frames', by_frame_id={}, reference_ray_y=None)

    ts = np.array([r[0] for r in raw_rows], float)
    rys = np.array([r[2] for r in raw_rows], float)
    ref_ry = float(np.median(rys))
    # Smooth over a 1.8s rolling window and clamp to +/-0.009 rad (~0.52 deg)
    smoothed_delta = np.zeros(len(raw_rows), float)
    for i, t in enumerate(ts):
        win = np.abs(ts - t) <= 0.9
        med_ry = float(np.median(rys[win]))
        smoothed_delta[i] = float(np.clip(ref_ry - med_ry, -0.009, 0.009))

    # Interpolate onto every front frame timestamp
    all_t = np.array([float(f['camera_time_s']) for f in front_frames], float)
    interp_delta = np.interp(all_t, ts, smoothed_delta)
    by_fid = {f['frame_id']: float(d) for f, d in zip(front_frames, interp_delta)}
    return dict(
        status='estimated_from_unoccluded_road_horizon',
        reference_ray_y=ref_ry,
        measured_frames=len(raw_rows),
        max_abs_pitch_rad=float(np.max(np.abs(interp_delta))),
        by_frame_id=by_fid,
    )


def project_front_ray_and_depths(row, profile, mount, vehicle_height, image_width=1280,
                                 image_height=960, contact_uv=None, delta_pitch_rad=0.0):
    """Compute bearing ray, pitch-compensated contact depth, and apparent-size depth."""
    b = row['bbox_xyxy']
    w = float(b[2] - b[0])
    h = float(b[3] - b[1])
    if w <= 2.0 or h <= 2.0:
        raise ValueError('degenerate_front_bbox')
    margin_x = float(min(b[0], image_width - b[2]))
    margin_all = float(min(b[0], b[1], image_width - b[2], image_height - b[3]))
    clear = (
        row.get('support') == 'detector_observation'
        and 'truncated' not in row.get('quality_flags', [])
        and h >= 25.0
        and w >= 25.0
        and margin_all >= 6.0
    )

    K, D = camera_matrix(profile)
    cam_h = float(mount['height_m'])
    cx = 0.5 * (float(b[0]) + float(b[2]))
    top_bot = cv2.undistortPoints(np.array([[[cx, float(b[1])]], [[cx, float(b[3])]]], float), K, D).reshape(2, 2)
    apparent_h = float(top_bot[1, 1] - top_bot[0, 1])
    ray_x_center = float(0.5 * (top_bot[0, 0] + top_bot[1, 0]))
    z_size = float(vehicle_height / apparent_h) if (np.isfinite(apparent_h) and apparent_h > 0.012 and clear) else None
    if z_size is not None and not (1.5 < z_size < 140.0):
        z_size = None

    if contact_uv is None:
        px = np.array([[[cx, float(b[3])]]], float)
    else:
        px = np.array([[[float(b[0]) + float(contact_uv[0]) * w, float(b[1]) + float(contact_uv[1]) * h]]], float)
    c_ray = cv2.undistortPoints(px, K, D).reshape(2)
    ray_x = float(c_ray[0])
    raw_ray_y = float(c_ray[1])

    # Ramp pitch correction smoothly above ~18m (raw_ray_y < 0.072) so near-field <18m stays exact
    pitch_ramp = float(np.clip((0.072 - raw_ray_y) / 0.036, 0.0, 1.0)) if delta_pitch_rad != 0.0 else 0.0
    corr_ray_y = raw_ray_y + pitch_ramp * float(delta_pitch_rad)

    z_contact = None
    if corr_ray_y > 0.003:
        zc_cand = cam_h / corr_ray_y
        if 0.4 < zc_cand < 140.0 and (zc_cand <= 70.0 or clear):
            z_contact = float(zc_cand)

    if z_contact is None and z_size is None:
        raise ValueError('horizon_or_invalid_contact')

    border_clipped = ('truncated' in row.get('quality_flags', [])) or margin_x < 10.0 or float(b[3]) > image_height - 6.0
    return dict(
        ray_x=ray_x if z_contact is not None else ray_x_center,
        z_contact=z_contact,
        z_size=z_size,
        clear=clear,
        border_clipped=border_clipped,
        learned_uv_used=contact_uv is not None,
    )


def fuse_front_track_depths(raw_front_entries, mount, half_length_m, rotation_fn, ego_times, ego_positions, ego_yaws):
    """Self-calibrate apparent vehicle size from clear mid-range contacts, then fuse depths."""
    # 1. Estimate per-track size calibration scale kappa from unoccluded 10..42m samples
    ratios = []
    for item in raw_front_entries:
        info = item['ray_info']
        zc, zs = info['z_contact'], info['z_size']
        if zc is None or zs is None or info['border_clipped'] or not info['clear']:
            continue
        if 10.0 <= zc <= 42.0 and abs(zc - zs) / max(zs, 1.0) <= 0.38:
            ratios.append(zc / zs)
    has_cal = len(ratios) >= 4
    kappa = float(np.clip(np.median(ratios), 0.78, 1.25)) if has_cal else 1.0

    mount_yaw = np.radians(float(mount['yaw_deg']))
    mount_off = np.asarray(mount['offset_W_ego_m'], float)
    R_mount = rotation_fn(mount_yaw)

    for item in raw_front_entries:
        info = item['ray_info']
        sample = item['sample']
        zc = info['z_contact']
        zs = info['z_size'] * kappa if info['z_size'] is not None else None
        rx = info['ray_x']

        if zc is not None and zs is not None:
            sig_c = (0.16 if info['learned_uv_used'] else 0.28) + 0.0022 * (zc * zc)
            if info['border_clipped']:
                sig_c *= 3.5
            # Occluded-bottom / horizon-grazing spike guard (e.g. zc=134m when box size says 27m)
            if zc > 1.42 * zs + 7.5:
                sig_c *= 4.5
            sig_s = (0.85 + 0.105 * zs) if has_cal else (1.25 + 0.155 * zs)
            # Keep near-field (<22m) non-clipped contacts pure unless occluded
            near_blend = float(np.clip((max(zc, zs) - 20.0) / 15.0, 0.0, 1.0))
            if info['border_clipped'] or zc > 1.42 * zs + 7.5:
                near_blend = 1.0
            prec_c = 1.0 / (sig_c * sig_c)
            prec_s = near_blend / (sig_s * sig_s)
            w_s = float(prec_s / (prec_c + prec_s))
            z_fused = (1.0 - w_s) * zc + w_s * zs
            sigma_fused = float(1.0 / math.sqrt(prec_c + prec_s))
            support = (
                'estimated_ground_contact' if z_fused <= 70.0 and w_s < 0.65
                else 'far_size_contact_uncertainty_fusion'
            )
        elif zc is not None:
            z_fused = zc
            sigma_fused = (0.22 + 0.0025 * (zc * zc)) * (2.5 if info['border_clipped'] else 1.0)
            support = 'estimated_ground_contact' if zc <= 70.0 else 'far_ground_contact_low_confidence'
        else:
            z_fused = zs
            sigma_fused = (0.95 + 0.12 * zs) if has_cal else (1.4 + 0.18 * zs)
            support = 'observed_height_prior_after_unresolved_contact'

        rel = R_mount @ np.array([z_fused, -z_fused * rx, 0.0]) + mount_off
        rel[0] += float(half_length_m)
        t = float(sample['time_s'])
        yaw = float(np.interp(t, ego_times, ego_yaws))
        ep = np.array([np.interp(t, ego_times, ego_positions[:, k]) for k in range(3)], float)
        pw = ep + rotation_fn(yaw) @ rel

        sample['position_W'] = pw.tolist()
        sample['unrefined_position_W'] = pw.tolist()
        sample['projection_support'] = support
        sample['position_sigma_m'] = float(sigma_fused)
        sample['quality'] = float(np.clip(1.0 / (1.0 + 0.35 * sigma_fused * sigma_fused), 0.18, 1.0))
        if info['border_clipped']:
            sample['quality'] = min(sample['quality'], 0.30)
    return kappa
