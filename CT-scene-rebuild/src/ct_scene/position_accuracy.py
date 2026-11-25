"""Step 1 baseline position accuracy & surrogate consistency evaluator.

Evaluates:
1. Supervised 3D tire-contact projection error on ``contact-learning-annotations``
   stratified by split, camera (front / left_repeater / right_repeater), and
   distance range (0-20m, 20-50m, 50m+).
2. Unlabeled full-scene surrogate accuracy metrics on ``surrounding-prototype``
   exports: cross-camera disagreement, trajectory jitter (raw vs rendered),
   carriageway out-of-bounds samples (raw unrefined vs final), implausible
   kinematics, and artificial clamp/collision displacement.
"""
from collections import defaultdict
from pathlib import Path
import json
import math
import numpy as np
from PIL import Image
from .artifacts import canonical
from .contact_learning import contact_target
from .junctions import inside as inside_junction
from .learned_contact import LearnedContact
from .road_frame import coordinates as road_coordinates
from .road_geometry import boundaries_at, pocket_width, supported as road_supported
from .surrounding import project_contact, project_vehicle_height

DISTANCE_BINS = (
    ('0_20m', 0.0, 20.0),
    ('20_50m', 20.0, 50.0),
    ('50m_plus', 50.0, float('inf')),
)


def distance_bin_name(dist_m):
    for name, lo, hi in DISTANCE_BINS:
        if lo <= dist_m < hi:
            return name
    return '50m_plus'


def _summary_stats(values):
    arr = np.asarray(values, float)
    arr = arr[np.isfinite(arr)]
    if not len(arr):
        return dict(count=0, mean_m=None, median_m=None, p90_m=None, max_m=None)
    return dict(
        count=int(len(arr)),
        mean_m=float(np.mean(arr)),
        median_m=float(np.median(arr)),
        p90_m=float(np.quantile(arr, 0.9)),
        max_m=float(np.max(arr)),
    )


def _crop_from_record(img_cache, record):
    src = Path(record['source_image'])
    key = str(src)
    if key not in img_cache:
        if len(img_cache) > 48:
            img_cache.clear()
        img_cache[key] = np.asarray(Image.open(src).convert('RGB'))
    im = img_cache[key]
    h, w = im.shape[:2]
    cr = np.asarray(record['crop_rect_xyxy'], float).round().astype(int)
    x1, y1 = max(0, int(cr[0])), max(0, int(cr[1]))
    x2, y2 = min(w, int(cr[2])), min(h, int(cr[3]))
    return im[y1:y2, x1:x2], [x1, y1, x2, y2]


def evaluate_contact_labels(annotations_path, lens_config_path, prototype_config_path,
                            adapter_dirs=None, default_vehicle_height_m=1.50):
    """Evaluate 3D ego-relative XY projection errors against annotated tire contacts."""
    ann = json.loads(Path(annotations_path).read_text(encoding='utf-8'))
    lens_cfg = json.loads(Path(lens_config_path).read_text(encoding='utf-8'))['cameras']
    proto_cfg = json.loads(Path(prototype_config_path).read_text(encoding='utf-8'))['cameras']
    adapters = {}
    for d in (adapter_dirs or []):
        dp = Path(d)
        adapters[dp.name] = LearnedContact(dp)

    img_cache = {}
    rows = []
    skipped_gt = 0
    for rec in ann.get('records', []):
        uv_gt = contact_target(rec)
        if uv_gt is None:
            continue
        cam = rec['camera']
        if cam not in lens_cfg or cam not in proto_cfg:
            continue
        profile = lens_cfg[cam]
        mount = proto_cfg[cam]
        box = rec['bbox_xyxy']
        try:
            p_gt = project_contact(box, profile, mount, max_depth=160., minimum_ray_y=.002, contact_uv=uv_gt)
        except ValueError:
            skipped_gt += 1
            continue
        dist_gt = float(np.linalg.norm(p_gt[:2]))
        dbin = distance_bin_name(dist_gt)
        preds = {}
        # 1. Raw bbox bottom midpoint (u=0.5, v=1.0)
        try:
            p_btm = project_contact(box, profile, mount, max_depth=160., minimum_ray_y=.002, contact_uv=[0.5, 1.0])
            preds['bbox_bottom'] = p_btm[:2]
        except ValueError:
            preds['bbox_bottom'] = None
        # 2. Apparent vehicle height prior (currently used on side repeaters)
        try:
            p_h = project_vehicle_height(box, profile, mount, default_vehicle_height_m)
            preds['apparent_height_prior'] = p_h[:2]
        except ValueError:
            preds['apparent_height_prior'] = None
        # 3. Learned adapters
        crop = None
        crop_rect = None
        if adapters and Path(rec.get('source_image', '')).is_file():
            crop, crop_rect = _crop_from_record(img_cache, rec)
        for name, model in adapters.items():
            if crop is None or crop.size == 0:
                preds[name] = None
                continue
            pred_rec = dict(camera=cam, bbox_xyxy=box, crop_rect_xyxy=crop_rect)
            uv_pred = model.predict(crop, pred_rec)['contact_uv']
            try:
                p_ad = project_contact(box, profile, mount, max_depth=160., minimum_ray_y=.002, contact_uv=uv_pred)
                preds[name] = p_ad[:2]
            except ValueError:
                preds[name] = None
        # 4. Current production pipeline simulation (front=v6 or bbox_bottom; side=apparent_height_prior)
        w_box, h_box = box[2] - box[0], box[3] - box[1]
        gate_ok = w_box >= 85 and h_box >= 55 and min(box[0], box[1], 1280 - box[2], 960 - box[3]) >= 8
        if cam == 'front':
            if 'contact-adapter-v6' in preds and gate_ok and preds['contact-adapter-v6'] is not None:
                preds['current_pipeline'] = preds['contact-adapter-v6']
            else:
                preds['current_pipeline'] = preds['bbox_bottom']
        else:
            preds['current_pipeline'] = preds['apparent_height_prior']

        rows.append(dict(
            id=rec['id'],
            split=rec['split'],
            session=rec['session'],
            camera=cam,
            distance_gt_m=dist_gt,
            distance_bin=dbin,
            gt_xy=p_gt[:2].tolist(),
            predictions={k: (v.tolist() if v is not None else None) for k, v in preds.items()},
        ))

    methods = ['current_pipeline', 'bbox_bottom', 'apparent_height_prior', *adapters.keys()]
    subsets = {
        'all_visible': lambda r: True,
        'holdout_val_test': lambda r: r['split'] in ('validation', 'test'),
        'untouched_test_only': lambda r: r['split'] == 'test',
    }
    summary = {}
    for subset_name, pred_fn in subsets.items():
        sub_rows = [r for r in rows if pred_fn(r)]
        method_out = {}
        for m in methods:
            errs_all, lon_all, lat_all = [], [], []
            by_cam = defaultdict(list)
            by_bin = defaultdict(list)
            by_cam_bin = defaultdict(list)
            unresolved = 0
            for r in sub_rows:
                p = r['predictions'].get(m)
                if p is None:
                    unresolved += 1
                    continue
                diff = np.asarray(p, float) - np.asarray(r['gt_xy'], float)
                err = float(np.linalg.norm(diff))
                errs_all.append(err)
                lon_all.append(abs(float(diff[0])))
                lat_all.append(abs(float(diff[1])))
                by_cam[r['camera']].append(err)
                by_bin[r['distance_bin']].append(err)
                by_cam_bin[f"{r['camera']}:{r['distance_bin']}"].append(err)
            method_out[m] = dict(
                overall=_summary_stats(errs_all),
                longitudinal_abs=_summary_stats(lon_all),
                lateral_abs=_summary_stats(lat_all),
                unresolved_count=unresolved,
                by_camera={c: _summary_stats(by_cam[c]) for c in ('front', 'left_repeater', 'right_repeater')},
                by_distance_bin={b: _summary_stats(by_bin[b]) for b, _, _ in DISTANCE_BINS},
                by_camera_and_bin={k: _summary_stats(v) for k, v in sorted(by_cam_bin.items())},
            )
        summary[subset_name] = dict(total_records=len(sub_rows), methods=method_out)

    return dict(
        annotations_path=str(annotations_path),
        visible_evaluated_records=len(rows),
        skipped_horizon_gt_records=skipped_gt,
        subsets=summary,
    )


def _second_diff_jitter(times, positions_xy, max_dt=0.26):
    """Local linear interpolation residual (meters) for consecutive triplets."""
    t = np.asarray(times, float)
    p = np.asarray(positions_xy, float)[:, :2]
    if len(t) < 3:
        return np.empty(0, float), np.empty(0, int)
    dt1 = t[1:-1] - t[:-2]
    dt2 = t[2:] - t[1:-1]
    valid = (dt1 > 1e-4) & (dt2 > 1e-4) & (dt1 <= max_dt) & (dt2 <= max_dt)
    idx = np.flatnonzero(valid) + 1
    if not len(idx):
        return np.empty(0, float), np.empty(0, int)
    w = (t[idx] - t[idx - 1]) / (t[idx + 1] - t[idx - 1])
    interp = (1.0 - w)[:, None] * p[idx - 1] + w[:, None] * p[idx + 1]
    res = np.linalg.norm(p[idx] - interp, axis=1)
    return res, idx


def evaluate_scene_surrogates(scene_dir):
    """Compute unlabeled surrogate position accuracy metrics on a rendered scene."""
    scene_dir = Path(scene_dir)
    report = json.loads((scene_dir / 'report.json').read_text(encoding='utf-8'))
    actors = json.loads((scene_dir / 'actors.json').read_text(encoding='utf-8'))
    states = [json.loads(l) for l in (scene_dir / 'render-states.jsonl').read_text(encoding='utf-8').splitlines()]
    edges_path = scene_dir / 'association-review.json'
    edges = json.loads(edges_path.read_text(encoding='utf-8')) if edges_path.is_file() else []
    road = report['road']

    ego_by_time = np.array([[s['time_s'], s['ego_position_W'][0], s['ego_position_W'][1]] for s in states], float)

    def ego_at(ts):
        return np.column_stack([
            np.interp(ts, ego_by_time[:, 0], ego_by_time[:, 1]),
            np.interp(ts, ego_by_time[:, 0], ego_by_time[:, 2]),
        ])

    # 1. Out-of-bounds & raw-vs-final displacement on corridor actors
    raw_oob = 0
    final_oob = 0
    final_body_overhang = 0
    corridor_samples = 0
    raw_to_final_shifts = []
    raw_oob_by_cam = defaultdict(int)
    final_oob_by_cam = defaultdict(int)
    worst_raw_oob_actors = []

    # 2. Jitter & kinematics on actor samples
    raw_jitter_all = []
    raw_jitter_by_cam = defaultdict(list)
    raw_jitter_by_bin = defaultdict(list)
    raw_speed_spikes = 0
    raw_lateral_spikes = 0
    raw_kinematic_pairs = 0

    # 3. Within-actor camera handoff jumps (unrefined)
    handoff_jumps = []

    for actor in actors:
        samples = actor.get('samples', [])
        if not samples:
            continue
        gid = actor.get('global_id', actor.get('id', 'unknown'))
        lane_idx = actor.get('lane_index')
        is_corridor_actor = (lane_idx is None and actor.get('frozen_lead')) or (isinstance(lane_idx, int) and lane_idx >= -1)

        ts = np.array([r['time_s'] for r in samples], float)
        pos_fin = np.array([r['position_W'][:2] for r in samples], float)
        pos_raw = np.array([r.get('unrefined_position_W', r['position_W'])[:2] for r in samples], float)
        ep = ego_at(ts)
        dists = np.linalg.norm(pos_fin - ep, axis=1)

        # Handoff unrefined jump when camera changes inside a merged global actor
        obs_idx = [i for i, r in enumerate(samples) if r.get('camera_id') in ('front', 'left_repeater', 'right_repeater')]
        for i0, i1 in zip(obs_idx, obs_idx[1:]):
            c0, c1 = samples[i0]['camera_id'], samples[i1]['camera_id']
            dt = ts[i1] - ts[i0]
            if c0 != c1 and 0.0 <= dt <= 2.5:
                # Extrapolate simple constant velocity from previous 3 samples of c0 if available
                prev = [k for k in obs_idx if k <= i0 and samples[k]['camera_id'] == c0 and ts[i0] - ts[k] <= 1.2]
                if len(prev) >= 2 and ts[prev[-1]] - ts[prev[0]] >= 0.2:
                    v_est = (pos_raw[prev[-1]] - pos_raw[prev[0]]) / (ts[prev[-1]] - ts[prev[0]])
                else:
                    v_est = np.zeros(2)
                pred_p = pos_raw[i0] + v_est * dt
                handoff_jumps.append(float(np.linalg.norm(pos_raw[i1] - pred_p)))

        # Raw jitter (only on real camera observations, excluding reconstructed gaps)
        real_mask = np.array([r.get('camera_id') in ('front', 'left_repeater', 'right_repeater') for r in samples], bool)
        if np.count_nonzero(real_mask) >= 3:
            r_ts = ts[real_mask]
            r_raw = pos_raw[real_mask]
            r_dists = dists[real_mask]
            r_cams = [samples[k]['camera_id'] for k in np.flatnonzero(real_mask)]
            jit, j_idx = _second_diff_jitter(r_ts, r_raw)
            for val, k in zip(jit, j_idx):
                raw_jitter_all.append(float(val))
                raw_jitter_by_cam[r_cams[k]].append(float(val))
                raw_jitter_by_bin[distance_bin_name(float(r_dists[k]))].append(float(val))
            for k in range(len(r_ts) - 1):
                dt = r_ts[k + 1] - r_ts[k]
                if 0.05 <= dt <= 0.35 and r_cams[k] == r_cams[k + 1]:
                    raw_kinematic_pairs += 1
                    spd = float(np.linalg.norm(r_raw[k + 1] - r_raw[k]) / dt)
                    if spd > 45.0:
                        raw_speed_spikes += 1

        if is_corridor_actor and (road.get('reference_line') or road.get('boundaries_y')):
            q_raw = road_coordinates(road, np.c_[pos_raw, np.zeros(len(pos_raw))])
            q_fin = road_coordinates(road, np.c_[pos_fin, np.zeros(len(pos_fin))])
            actor_raw_oob = 0
            for k, r in enumerate(samples):
                if r.get('camera_id') == 'reconstructed':
                    continue
                if inside_junction(road, pos_fin[k], 3.0):
                    continue
                s_f = float(q_fin[k, 0])
                if road.get('reference_line') and not road_supported(road, s_f):
                    continue
                corridor_samples += 1
                cam = r.get('camera_id') or 'unknown'
                b_raw = boundaries_at(road, float(q_raw[k, 0]))
                pw_raw = pocket_width(road.get('left_pockets', []), float(q_raw[k, 0]))
                if q_raw[k, 1] < b_raw[-1] or q_raw[k, 1] > b_raw[0] + pw_raw:
                    raw_oob += 1
                    actor_raw_oob += 1
                    raw_oob_by_cam[cam] += 1
                b_fin = boundaries_at(road, s_f)
                pw_fin = pocket_width(road.get('left_pockets', []), s_f)
                if q_fin[k, 1] < b_fin[-1] or q_fin[k, 1] > b_fin[0] + pw_fin:
                    final_oob += 1
                    final_oob_by_cam[cam] += 1
                if q_fin[k, 1] < b_fin[-1] + 0.75 or q_fin[k, 1] > b_fin[0] + pw_fin - 0.75:
                    final_body_overhang += 1
                raw_to_final_shifts.append(float(np.linalg.norm(pos_fin[k] - pos_raw[k])))
            if actor_raw_oob > 0:
                worst_raw_oob_actors.append(dict(id=gid, raw_oob_samples=actor_raw_oob, total_samples=len(samples)))
            # Check raw lateral velocity spikes in road frame
            if np.count_nonzero(real_mask) >= 2:
                rq = q_raw[real_mask]
                rt = ts[real_mask]
                rc = [samples[k]['camera_id'] for k in np.flatnonzero(real_mask)]
                for k in range(len(rt) - 1):
                    dt = rt[k + 1] - rt[k]
                    if 0.05 <= dt <= 0.35 and rc[k] == rc[k + 1]:
                        if abs(float(rq[k + 1, 1] - rq[k, 1])) / dt > 6.0:
                            raw_lateral_spikes += 1

    # 4. Rendered 20 Hz frame-to-frame jitter, implausible kinematics & collision displacement
    render_tracks = defaultdict(list)
    collision_shifted_frames = 0
    collision_shifts = []
    coobserved_close_conflicts = []

    # Map actor primary camera for simultaneous front-vs-side split-track detection
    actor_cams = {a['global_id']: {r.get('camera_id') for r in a['samples'] if r.get('camera_id') in ('front', 'left_repeater', 'right_repeater')} for a in actors}

    for st in states:
        t = st['time_s']
        st_actors = st['actors']
        for a in st_actors:
            render_tracks[a['id']].append((t, np.asarray(a['position_W'][:2], float), float(a['yaw_rad'])))
            if 'collision_correction_W' in a:
                c_shift = float(np.linalg.norm(a['collision_correction_W'][:2]))
                if c_shift > 0.05:
                    collision_shifted_frames += 1
                    collision_shifts.append(c_shift)
        # Detect simultaneous unmerged front-only vs side-only tracks within 4.5m XY (split-track / co-observation disagreement)
        for i, a in enumerate(st_actors):
            if a.get('ghost'):
                continue
            ca = actor_cams.get(a['id'], set())
            for b in st_actors[i + 1:]:
                if b.get('ghost'):
                    continue
                cb = actor_cams.get(b['id'], set())
                if ('front' in ca and cb <= {'left_repeater', 'right_repeater'}) or ('front' in cb and ca <= {'left_repeater', 'right_repeater'}):
                    pa = np.asarray(a.get('position_before_collision_W', a['position_W'])[:2], float)
                    pb = np.asarray(b.get('position_before_collision_W', b['position_W'])[:2], float)
                    dist_ab = float(np.linalg.norm(pa - pb))
                    if dist_ab < 5.5:
                        coobserved_close_conflicts.append(dist_ab)

    rendered_jitter_all = []
    rendered_speed_spikes = 0
    rendered_accel_spikes = 0
    rendered_step_count = 0
    for gid, rows in render_tracks.items():
        if len(rows) < 3:
            continue
        rt = np.array([x[0] for x in rows], float)
        rp = np.array([x[1] for x in rows], float)
        jit, _ = _second_diff_jitter(rt, rp, max_dt=0.11)
        rendered_jitter_all.extend(jit.tolist())
        dts = np.diff(rt)
        vels = np.diff(rp, axis=0) / np.maximum(dts[:, None], 1e-4)
        spds = np.linalg.norm(vels, axis=1)
        valid_step = dts <= 0.11
        rendered_step_count += int(np.count_nonzero(valid_step))
        rendered_speed_spikes += int(np.count_nonzero(valid_step & (spds > 42.0)))
        if len(vels) >= 2:
            acc = np.linalg.norm(np.diff(vels, axis=0), axis=1) / np.maximum(dts[1:], 1e-4)
            valid_acc = (dts[:-1] <= 0.11) & (dts[1:] <= 0.11)
            rendered_accel_spikes += int(np.count_nonzero(valid_acc & (acc > 12.0)))

    selected_cross_edges = [e['distance_m_prior'] for e in edges if e.get('selected') and e.get('kind') != 'same_camera_box_continuity' and 'distance_m_prior' in e]
    fov_candidate_edges = [e['distance_m_prior'] for e in edges if e.get('kind') == 'future_reappearance_with_fov_boundary' and 'distance_m_prior' in e]

    worst_raw_oob_actors.sort(key=lambda x: -x['raw_oob_samples'])
    return dict(
        scene=scene_dir.name,
        global_actors=len(actors),
        render_frames=len(states),
        cross_camera_disagreement=dict(
            selected_handoff_edge_error=_summary_stats(selected_cross_edges),
            fov_boundary_candidate_error=_summary_stats(fov_candidate_edges),
            merged_actor_handoff_jump=_summary_stats(handoff_jumps),
            simultaneous_front_side_close_conflicts=dict(
                actor_pair_frames=len(coobserved_close_conflicts),
                distance=_summary_stats(coobserved_close_conflicts),
            ),
        ),
        trajectory_jitter=dict(
            raw_projection_jitter=_summary_stats(raw_jitter_all),
            raw_by_camera={c: _summary_stats(raw_jitter_by_cam[c]) for c in ('front', 'left_repeater', 'right_repeater')},
            raw_by_distance_bin={b: _summary_stats(raw_jitter_by_bin[b]) for b, _, _ in DISTANCE_BINS},
            rendered_20hz_jitter=_summary_stats(rendered_jitter_all),
        ),
        carriageway_bounds=dict(
            evaluated_corridor_samples=corridor_samples,
            raw_unrefined_out_of_bounds_samples=raw_oob,
            raw_unrefined_out_of_bounds_fraction=float(raw_oob / max(1, corridor_samples)),
            raw_out_of_bounds_by_camera=dict(raw_oob_by_cam),
            final_center_out_of_bounds_samples=final_oob,
            final_body_overhang_samples=final_body_overhang,
            final_out_of_bounds_by_camera=dict(final_oob_by_cam),
            worst_raw_oob_actors=worst_raw_oob_actors[:6],
        ),
        kinematic_plausibility=dict(
            raw_observation_pairs=raw_kinematic_pairs,
            raw_speed_gt_45mps_samples=raw_speed_spikes,
            raw_lateral_speed_gt_6mps_samples=raw_lateral_spikes,
            rendered_20hz_steps=rendered_step_count,
            rendered_speed_gt_42mps_frames=rendered_speed_spikes,
            rendered_accel_gt_12mps2_frames=rendered_accel_spikes,
        ),
        artificial_displacement=dict(
            raw_to_final_shift=_summary_stats(raw_to_final_shifts),
            collision_shifted_actor_frames=collision_shifted_frames,
            collision_shift=_summary_stats(collision_shifts),
            collision_overlap_frames_before=report.get('collision_constraints', {}).get('before', {}).get('overlapping_pair_frames'),
            collision_overlap_frames_after=report.get('collision_constraints', {}).get('after', {}).get('overlapping_pair_frames'),
        ),
    )


def run(output, annotations=None, lens_config='configs/multicam-optical-candidates.json',
        prototype_config='configs/surrounding-prototype.json', adapters=None, scenes=None):
    """Run Step 1 position accuracy benchmark and save canonical JSON report."""
    payload = dict(
        schema='ct_scene.position_accuracy_baseline_v1',
        contact_label_benchmark=None,
        scene_surrogates={},
    )
    if annotations:
        payload['contact_label_benchmark'] = evaluate_contact_labels(
            annotations, lens_config, prototype_config, adapter_dirs=adapters or []
        )
    for sc in (scenes or []):
        sc_res = evaluate_scene_surrogates(sc)
        payload['scene_surrogates'][Path(sc).name] = sc_res

    out_dir = Path(output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / 'position-accuracy.json'
    out_file.write_bytes(canonical(payload))
    return payload
