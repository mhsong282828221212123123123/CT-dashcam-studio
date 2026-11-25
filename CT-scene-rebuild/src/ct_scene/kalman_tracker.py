"""Step 4 unified per-vehicle road-frame Kalman filter & RTS smoother.

Fuses multi-camera observations (including simultaneous front + side overlap and
cross-camera handoffs) using each observation's physical covariance ``R_k``,
incorporates camera-supported soft lane likelihoods and returns conditional
smoothing uncertainty. Camera/class calibration is estimated, so this is not a
surveyed confidence region or an independent metric-accuracy guarantee.
"""
from collections import defaultdict
import math
import numpy as np
from .actor_lane_motion import observation_usable
from .camera_covariance import ray_covariance,update_position,initialize_offline
from .crossing_smoother import smooth_crossing,bearing_conflict
from .road_boundary_confidence import lane_confidence,membership_supported
from .junctions import inside as inside_junction
from .road_frame import coordinates as road_coordinates, world as road_world
from .road_geometry import boundaries_at, lane_at, lane_bounds, pocket_width, supported as road_supported


def _transition(dt, tau_d=1.8):
    decay = math.exp(-dt / tau_d) if tau_d > 0 else 1.0
    d_int = tau_d * (1.0 - decay) if tau_d > 0 else dt
    F = np.array([
        [1.0, 0.0, dt,    0.0],
        [0.0, 1.0, 0.0, d_int],
        [0.0, 0.0, 1.0,   0.0],
        [0.0, 0.0, 0.0, decay],
    ], float)
    return F


def _process_cov(dt, sigma_as=2.8, sigma_ad=1.15):
    dt = max(float(dt), 1e-3)
    qs = sigma_as * sigma_as
    qd = sigma_ad * sigma_ad
    dt2 = dt * dt
    dt3 = dt2 * dt
    Q = np.array([
        [qs * dt3 / 3.0, 0.0,            qs * dt2 / 2.0, 0.0],
        [0.0,            qd * dt3 / 3.0, 0.0,            qd * dt2 / 2.0],
        [qs * dt2 / 2.0, 0.0,            qs * dt,        0.0],
        [0.0,            qd * dt2 / 2.0, 0.0,            qd * dt],
    ], float)
    return Q


def observation_cov_sd(row, dist_to_ego_m):
    """Anisotropic (sigma_s, sigma_d) in road frame from camera, range, and occlusion."""
    cam = row.get('camera_id')
    base_sig = row.get('position_sigma_m')
    sup = row.get('projection_support', '')
    trunc = 'truncated' in (row.get('quality_flags') or []) or row.get('_is_truncated', False)

    if cam == 'reconstructed' or sup == 'offline_two_endpoint_gap':
        return 2.2, 1.4

    if base_sig is not None and np.isfinite(base_sig):
        sig = float(base_sig)
    else:
        if cam == 'front':
            sig = 0.25 + 0.0022 * (dist_to_ego_m ** 2)
        else:
            sig = (1.3 + 0.25 * dist_to_ego_m) if sup == 'apparent_vehicle_height_prior' else (0.35 + 0.004 * (dist_to_ego_m ** 2))

    if cam == 'front':
        # Front camera has very tight angular bearing (|ray_x| ~ 0.05..0.15) and softer depth
        sig_s = max(0.28, sig)
        sig_d = max(0.18, min(1.35, 0.14 * sig + 0.16))
    elif cam in ('left_repeater', 'right_repeater'):
        if sup == 'apparent_vehicle_height_prior':
            sig_s = max(1.2, 0.85 * sig)
            sig_d = max(1.1, 0.75 * sig)
        else:
            sig_s = max(0.35, 0.85 * sig)
            sig_d = max(0.28, 0.65 * sig)
    else:
        sig_s = max(0.5, sig)
        sig_d = max(0.4, 0.7 * sig)

    if trunc:
        sig_s *= 1.8
        sig_d *= 1.6
    return float(sig_s), float(sig_d)


def _kalman_update_2d(x, P, z_sd, var_s, var_d):
    H = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]], float)
    R = np.array([[var_s, 0.0], [0.0, var_d]], float)
    y = np.asarray(z_sd, float) - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    I_KH = np.eye(4) - K @ H
    x_new = x + K @ y
    # Joseph stabilized covariance update
    P_new = I_KH @ P @ I_KH.T + K @ R @ K.T
    return x_new, 0.5 * (P_new + P_new.T)


def _kalman_update_d_only(x, P, z_d, var_d):
    H = np.array([[0.0, 1.0, 0.0, 0.0]], float)
    y = float(z_d) - float(x[1])
    S = float((H @ P @ H.T)[0, 0]) + float(var_d)
    K = (P @ H.T)[:, 0] / max(S, 1e-9)
    I_KH = np.eye(4) - np.outer(K, H[0])
    x_new = x + K * y
    P_new = I_KH @ P @ I_KH.T + np.outer(K, K) * float(var_d)
    return x_new, 0.5 * (P_new + P_new.T)


def _extended_corridor_station(road, s):
    """Clamp station to supported_extent_m for vehicles ahead of ego at clip ends."""
    if road_supported(road, s):
        return float(s), True
    if any(j['road_station_range_m'][0] < s < j['road_station_range_m'][1] for j in road.get('junctions', [])):
        return float(s), False
    line = road.get('reference_line')
    if not line:
        return float(s), False
    if any(a < s < b for a, b in line.get('unsupported_intervals_m', [])):
        return float(s), False
    ext = line.get('supported_extent_m')
    if ext and (ext[0] - 90.0 <= s < ext[0] or ext[1] < s <= ext[1] + 90.0):
        s_clamped = float(np.clip(s, ext[0] + 1e-3, ext[1] - 1e-3))
        if road_supported(road, s_clamped):
            return s_clamped, True
    return float(s), False


def limit_yaw_rate(actor, max_rate=1.4):
    """Bidirectional yaw-rate limiter: removes 1-frame heading snaps (vehicles that visibly spin)."""
    if actor.get('frozen_lead'):
        return 0
    rows = actor.get('samples', [])
    if len(rows) < 3:
        return 0
    t = np.array([r['time_s'] for r in rows], float)
    y = np.unwrap(np.array([r['yaw_rad'] for r in rows], float))
    dt = np.maximum(np.diff(t), 1e-3)
    f = y.copy()
    for k in range(1, len(y)):
        lim = max_rate * dt[k - 1]
        f[k] = f[k - 1] + np.clip(y[k] - f[k - 1], -lim, lim)
    b = y.copy()
    for k in range(len(y) - 2, -1, -1):
        lim = max_rate * dt[k]
        b[k] = b[k + 1] + np.clip(y[k] - b[k + 1], -lim, lim)
    out = 0.5 * (f + b)
    changed = 0
    for r, v0, v1 in zip(rows, y, out):
        if abs(v1 - v0) > 1e-6:
            r['yaw_before_rate_limit'] = float(r['yaw_rad'])
            r['yaw_rad'] = float(v1)
            changed += 1
    return changed


def smooth_actor_rts(actor, road, ego_times, ego_positions, raw_member_rows=None):
    """Run multi-camera road-frame Kalman filter + RTS smoother on a global actor."""
    if actor.get('frozen_lead'):
        return dict(applied=False, reason='frozen_lead_preserved')
    samples = actor.get('samples', [])
    if len(samples) < 3:
        return dict(applied=False, reason='too_few_samples')

    use_road_frame = bool(road and road.get('reference_line'))
    # Physical speed ceiling from the ego's own speed in this clip (never a free-flying vehicle)
    try:
        ego_sp = np.linalg.norm(np.gradient(np.asarray(ego_positions, float)[:, :2], np.asarray(ego_times, float), axis=0), axis=1)
        v_cap = float(np.clip(1.25 * float(np.percentile(ego_sp, 95)) + 6.0, 14.0, 36.0))
    except Exception:
        v_cap = 36.0
    lane_idx_actor = actor.get('lane_index')
    is_crossing = lane_idx_actor == -2
    # A crossing car is independent of the ego road nearest-segment projection.
    use_road_frame = use_road_frame and not is_crossing

    # Group all real camera observations by 20Hz/10Hz timestamp bin (0.05s)
    obs_pool = [r for r in (raw_member_rows if raw_member_rows is not None else samples)
                if r.get('camera_id') in ('front', 'left_repeater', 'right_repeater') and r.get('support') in ('detector_observation','flow_observation')]
    if len(obs_pool) < 2:
        return dict(applied=False, reason='no_camera_observations')

    obs_by_bin = defaultdict(list)
    for r in obs_pool:
        key_t = round(float(r['time_s']) * 20.0) / 20.0
        obs_by_bin[key_t].append(r)

    grid_times = np.array([round(float(s['time_s']) * 20.0) / 20.0 for s in samples], float)
    # Ensure strictly increasing unique grid matching samples
    if np.any(np.diff(grid_times) <= 0):
        grid_times = np.array([float(s['time_s']) for s in samples], float)

    def to_frame(pts_xy):
        arr = np.asarray(pts_xy, float)
        if use_road_frame:
            q = road_coordinates(road, np.c_[arr[:, :2], np.zeros(len(arr))])
            return q[:, :2], q[:, 2]
        return arr[:, :2], np.zeros(len(arr))

    def from_frame(sd_arr):
        sd_arr = np.asarray(sd_arr, float)
        if use_road_frame:
            return road_world(road, sd_arr[:, 0], sd_arr[:, 1])[:, :2]
        return sd_arr[:, :2]

    # Precompute observation (s, d) and (sigma_s, sigma_d)
    step_obs = [];world_step_obs=[];bearing_guard_obs=[]
    for k, s_row in enumerate(samples):
        tk = float(s_row['time_s'])
        bkey = round(tk * 20.0) / 20.0
        cands = obs_by_bin.get(bkey, [])
        if not cands and s_row.get('camera_id') in ('front', 'left_repeater', 'right_repeater') and s_row.get('support') in ('detector_observation','flow_observation'):
            cands = [s_row]
        ep = np.array([np.interp(tk, ego_times, ego_positions[:, 0]),
                       np.interp(tk, ego_times, ego_positions[:, 1])], float)
        parsed = [];world_parsed=[];guard_parsed=[]
        for r in cands:
            # Use raw unrefined camera observation so Kalman replaces ad-hoc clamps cleanly
            p_raw = np.asarray(r.get('unrefined_position_W', r['position_W'])[:2], float)
            sd, road_heading = to_frame(p_raw[None, :])
            dist = float(np.linalg.norm(p_raw - ep))
            sig_s, sig_d = observation_cov_sd(r, dist)
            world_cov,road_cov=ray_covariance(p_raw,ep,sig_s,sig_d,float(road_heading[0]))
            world_row=(p_raw,sig_s*sig_s,sig_d*sig_d,r.get('camera_id'),world_cov)
            world_parsed.append(world_row)
            box=r.get('bbox_xyxy')
            if (r.get('camera_id')=='front' and r.get('support')=='detector_observation' and box is not None
                    and box[2]-box[0]>=30 and box[3]-box[1]>=25
                    and min(box[0],box[1],r.get('image_width',1280)-box[2],r.get('image_height',960)-box[3])>6):
                guard_parsed.append(world_row)
            parsed.append((sd[0],float(road_cov[0,0]),float(road_cov[1,1]),r.get('camera_id'),world_cov if is_crossing else road_cov))
        step_obs.append(parsed);world_step_obs.append(world_parsed);bearing_guard_obs.append(guard_parsed)

    if is_crossing:
        return smooth_crossing(actor,samples,step_obs)
    if (actor.get('lane_motion_evidence',{}).get('clear_observations')==0
            or len({r.get('camera_id') for r in obs_pool})>=2
            or not any(membership_supported(actor,row) for row in samples)):
        # A measured lane can remain a weak geometric likelihood in the world
        # fit. Unresolved/disabled membership never acquires this likelihood.
        acceleration_precisions=np.tile(np.eye(2)/9.,(len(samples),1,1))
        if use_road_frame:
            for k,row in enumerate(samples):
                if not world_step_obs[k] or not membership_supported(actor,row):continue
                raw=np.mean([o[0] for o in world_step_obs[k]],axis=0)
                q=road_coordinates(road,np.array([[*raw,0.]]))[0];s_eff,s_sup=_extended_corridor_station(road,float(q[0]))
                lane=row.get('lane_index',lane_idx_actor)
                if lane is None:lane=lane_idx_actor
                if not s_sup or not isinstance(lane,int) or lane<0 or lane>=len(boundaries_at(road,s_eff))-1:continue
                confidence=lane_confidence(road,s_eff,lane)
                if confidence<.5 or inside_junction(road,raw,3.):continue
                center=sum(lane_bounds(road,s_eff,lane))/2
                target=road_world(road,float(q[0]),center)[:2]
                c,s=math.cos(q[2]),math.sin(q[2]);basis=np.array([[c,-s],[s,c]])
                acceleration_precisions[k]=basis@np.diag([1/9.,1/(1.1/max(confidence,.15))**2])@basis.T
                cov=basis@np.diag([1e8,(1.15/max(confidence,.15))**2])@basis.T
                world_step_obs[k].append((target,1e8,(1.15/confidence)**2,'paint_boundary_prior',cov))
        return smooth_crossing(actor,samples,world_step_obs,coordinate_frame='world_xy_multicamera_or_unresolved_lane',acceleration_precisions=acceleration_precisions)

    # Initialize state from first two observed steps
    first_idx = next((i for i, o in enumerate(step_obs) if o), 0)
    z0,v0,initial_position_var=initialize_offline(samples,step_obs,first_idx,v_cap)

    n = len(samples)
    x_pred = np.zeros((n, 4), float)
    P_pred = np.zeros((n, 4, 4), float)
    x_filt = np.zeros((n, 4), float)
    P_filt = np.zeros((n, 4, 4), float)
    F_list = [np.eye(4) for _ in range(n)]

    x_cur = np.array([z0[0], z0[1], v0[0], v0[1]], float)
    P_cur = np.diag([max(2.5,initial_position_var[0]),max(1.5,initial_position_var[1]),25.0,6.0])
    multi_cam_fused_steps = 0

    for k in range(n):
        tk = float(samples[k]['time_s'])
        if k == 0:
            x_p, P_p = x_cur.copy(), P_cur.copy()
        else:
            dt = max(1e-3, tk - float(samples[k - 1]['time_s']))
            in_junc = use_road_frame and inside_junction(road, from_frame(x_cur[:2][None, :])[0], 4.0)
            F = _transition(dt, tau_d=0.0 if (is_crossing or in_junc) else 1.8)
            Q = _process_cov(dt, sigma_as=3.0, sigma_ad=2.4 if (is_crossing or in_junc) else 1.1)
            F_list[k] = F
            x_p = F @ x_cur
            P_p = F @ P_cur @ F.T + Q

        x_pred[k] = x_p
        P_pred[k] = P_p
        x_u, P_u = x_p.copy(), P_p.copy()

        cams_at_k = {o[3] for o in step_obs[k]}
        if len(cams_at_k) >= 2:
            multi_cam_fused_steps += 1

        # Fuse all camera measurements at timestamp tk
        dt_step = max(0.05, tk - float(samples[k - 1]['time_s'])) if k > 0 else 0.05
        for z_sd, var_s, var_d, _, obs_cov in step_obs[k]:
            # Chi-square (2-DOF 99% = 9.21) + kinematic jump gate against end-of-track ID glitches
            innov = z_sd - x_u[:2]
            R_obs=np.diag([var_s,var_d]) if obs_cov is None else np.array(obs_cov,copy=True)
            mahal=float(innov@np.linalg.solve(P_u[:2,:2]+R_obs+np.eye(2)*1e-8,innov))
            kin_jump = k >= 1 and (abs(innov[0]) > max(12.0, 32.0 * dt_step) or abs(innov[1]) > max(3.2, 12.0 * dt_step))
            if k >= 2 and (mahal > 9.21 or kin_jump):
                scale_infl = min(400.0, max((mahal / 3.0) ** 2, 180.0 if kin_jump else 1.0))
                var_s *= scale_infl
                var_d *= scale_infl
                R_obs *= scale_infl
            x_u, P_u = update_position(x_u,P_u,z_sd,R_obs)

        # Soft lane-center likelihood for supported corridor vehicles outside junctions
        s_row = samples[k]
        sl = s_row.get('lane_index')
        if sl is None and isinstance(lane_idx_actor, int) and lane_idx_actor >= -1:
            sl = lane_idx_actor
        s_eff, s_sup = _extended_corridor_station(road, float(x_u[0])) if use_road_frame else (float(x_u[0]), True)
        if (not is_crossing and sl is not None and isinstance(sl, int) and sl >= -1
                and use_road_frame and s_sup and membership_supported(actor,s_row)
                and not inside_junction(road, from_frame(x_u[:2][None, :])[0], 3.0)):
            b_here = boundaries_at(road, s_eff)
            if sl == -1 and pocket_width(road.get('left_pockets', []), s_eff) >= 1.5:
                l_hi, l_lo = lane_bounds(road, s_eff, -1)
                d_center = 0.5 * (l_hi + l_lo)
                var_lane = 0.95 ** 2 / max(.15,lane_confidence(road,s_eff,-1))**2
                x_u, P_u = _kalman_update_d_only(x_u, P_u, d_center, var_lane)
            elif 0 <= sl < len(b_here) - 1:
                l_hi, l_lo = lane_bounds(road, s_eff, sl)
                d_center = 0.5 * (l_hi + l_lo)
                # Uncertain observations cannot strengthen a guessed lane. Repeated paint
                # determines the strength of this separate geometric likelihood.
                usable = observation_usable(s_row) if s_row.get('camera_id') in ('front', 'left_repeater', 'right_repeater') else False
                var_lane = (1.15 ** 2 if usable else 2.0 ** 2) / max(.15,lane_confidence(road,s_eff,sl))**2
                # Gate: a lane prior never drags a vehicle the cameras place far from that lane
                # (e.g. cross-street traffic mislabelled with a corridor lane).
                gate = max(2.4, 3.0 * math.sqrt(max(float(P_u[1, 1]), 0.0) + var_lane))
                if abs(d_center - float(x_u[1])) <= gate:
                    x_u, P_u = _kalman_update_d_only(x_u, P_u, d_center, var_lane)

        # Hard physical ceiling on state speed (prevents runaway velocity accumulation)
        spd = math.hypot(float(x_u[2]), float(x_u[3]))
        if spd > v_cap:
            x_u[2] *= v_cap / spd
            x_u[3] *= v_cap / spd

        x_filt[k] = x_u
        P_filt[k] = P_u
        x_cur, P_cur = x_u, P_u

    # Backward Rauch-Tung-Striebel (RTS) fixed-interval smoother
    x_sm = x_filt.copy()
    P_sm = P_filt.copy()
    for k in range(n - 2, -1, -1):
        P_p_next = P_pred[k + 1] + np.eye(4) * 1e-8
        G = P_filt[k] @ F_list[k + 1].T @ np.linalg.inv(P_p_next)
        x_sm[k] = x_filt[k] + G @ (x_sm[k + 1] - x_pred[k + 1])
        P_k = P_filt[k] + G @ (P_sm[k + 1] - P_pred[k + 1]) @ G.T
        P_sm[k] = 0.5 * (P_k + P_k.T)

    # Covariance-bounded soft carriageway projection (continuous across clip-end horizon)
    if not is_crossing and isinstance(lane_idx_actor, int) and lane_idx_actor >= -1 and use_road_frame:
        for k in range(n):
            if not membership_supported(actor,samples[k]):continue
            sk = float(x_sm[k, 0])
            dk = float(x_sm[k, 1])
            s_eff, s_sup = _extended_corridor_station(road, sk)
            if not s_sup or lane_confidence(road,s_eff,lane_idx_actor)<.5:
                continue
            if inside_junction(road, from_frame(np.array([[sk, dk]]))[0], 3.0):
                continue
            b_k = boundaries_at(road, s_eff)
            pw_k = pocket_width(road.get('left_pockets', []), s_eff)
            lo_edge = float(b_k[-1]) + 0.82
            hi_edge = float(b_k[0]) + pw_k - 0.82
            if hi_edge <= lo_edge:
                continue
            sig_d_post = math.sqrt(max(float(P_sm[k, 1, 1]), 0.04))
            max_move = max(0.65, 2.8 * sig_d_post)
            if dk < lo_edge:
                x_sm[k, 1] = min(lo_edge, dk + max_move)
            elif dk > hi_edge:
                x_sm[k, 1] = max(hi_edge, dk - max_move)

    xy_sm = from_frame(x_sm[:, :2])
    # An inherited lane without usable measurements cannot overrule a sequence
    # of precise camera bearings. Refit jointly instead of clipping or hiding.
    if actor.get('lane_motion_evidence',{}).get('clear_observations')==0:
        conflict=bearing_conflict(xy_sm,bearing_guard_obs,[r['time_s'] for r in samples])
        if conflict['trigger']:
            result=smooth_crossing(actor,samples,world_step_obs,coordinate_frame='world_xy_unresolved_lane')
            result['camera_bearing_fallback']=conflict
            for row in samples:
                row['lane_index']=None;row['lane_prior']='disabled_unresolved_lane_conflicts_with_direct_camera_bearing'
            return result

    if use_road_frame:
        headings_road = road_coordinates(road, np.c_[xy_sm, np.zeros(n)])[:, 2]
    else:
        headings_road = np.zeros(n)

    last_yaw = float(samples[0].get('yaw_rad', headings_road[0]))
    for k, s_row in enumerate(samples):
        z_orig = float(s_row['position_W'][2]) if len(s_row['position_W']) >= 3 else 0.0
        s_row['position_W'] = [float(xy_sm[k, 0]), float(xy_sm[k, 1]), z_orig]
        sig_s_post = math.sqrt(max(float(P_sm[k, 0, 0]), 0.02))
        sig_d_post = math.sqrt(max(float(P_sm[k, 1, 1]), 0.02))
        s_row['kalman_sigma_s_m'] = float(sig_s_post)
        s_row['kalman_sigma_d_m'] = float(sig_d_post)
        s_row['kalman_sigma_m'] = float(math.hypot(sig_s_post, sig_d_post))

        vs, vd = float(x_sm[k, 2]), float(x_sm[k, 3])
        speed = math.hypot(vs, vd)
        if use_road_frame and not is_crossing and not inside_junction(road, xy_sm[k], 3.0):
            if vs > 1.2:
                rel_ang = float(np.clip(math.atan2(vd, vs), -math.pi / 5.0, math.pi / 5.0))
                last_yaw = float(headings_road[k] + rel_ang)
            else:
                last_yaw = float(headings_road[k])
            s_row['yaw_rad'] = last_yaw
            if road_supported(road, float(x_sm[k, 0])):
                m_lane = lane_at(road, float(x_sm[k, 0]), float(x_sm[k, 1]))
                if m_lane is not None:
                    s_row['lane_index'] = int(m_lane)
        elif speed > 1.8 and not use_road_frame:
            last_yaw = float(math.atan2(vd, vs))
            s_row['yaw_rad'] = last_yaw

    return dict(
        applied=True,
        coordinate_frame='road_sd' if use_road_frame else 'world_xy',
        samples=n,
        multi_camera_fused_steps=multi_cam_fused_steps,
        median_posterior_sigma_m=float(np.median([s['kalman_sigma_m'] for s in samples])),
    )
