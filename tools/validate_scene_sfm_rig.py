"""Leave one synchronized camera pair out of rigid bundle adjustment.

The held-out right camera is predicted from a left-only localization. Its pose
is never fitted to right pixels. Units remain arbitrary SfM units.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root', type=Path)
    ap.add_argument('--time', default='022.0.jpg')
    ap.add_argument('--variant', choices=['original', 'robust', 'radial2', 'principal'], default='original')
    ap.add_argument('--model-id', default='1')
    args = ap.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scene-runtime-calibration'))
    import pycolmap as pc

    root = args.root
    out = root / ('heldout-' + args.time.replace('.jpg', '') + ('' if args.variant=='original' else '-'+args.variant))
    out.mkdir(exist_ok=True)
    rec = pc.Reconstruction(root / 'sparse' / args.model_id)
    saved = {}
    for im in list(rec.images.values()):
        if im.name.endswith('/' + args.time):
            entries = [(np.array(pt.xy), pt.point3D_id) for pt in im.points2D if pt.has_point3D()]
            saved[im.name.split('/')[0]] = entries
            rec.deregister_frame(im.frame_id)
    if set(saved) != {'left_repeater', 'right_repeater'}:
        raise ValueError('Both synchronized held-out images must be reconstructed')
    if args.variant == 'radial2':
        for camera in rec.cameras.values():
            if camera.model_name != 'SIMPLE_RADIAL':
                raise ValueError('radial2 variant expects original SIMPLE_RADIAL cameras')
            params = camera.params.copy()
            camera.model = pc.CameraModelId.RADIAL
            camera.params = np.r_[params, 0.]
    dbpath = out / 'rig.db'
    shutil.copyfile(root / 'database.db', dbpath)
    cfg = pc.RigConfig(cameras=[
        pc.RigConfigCamera(ref_sensor=True, image_prefix='left_repeater/'),
        pc.RigConfigCamera(ref_sensor=False, image_prefix='right_repeater/'),
    ])
    db = pc.Database.open(dbpath)
    pc.apply_rig_config([cfg], db, rec)
    db.close()
    # apply_rig_config must not accidentally put held-out observations back.
    assert not any(im.name.endswith('/' + args.time) for im in rec.images.values() if im.has_pose)
    opt = pc.BundleAdjustmentOptions()
    opt.print_summary = False
    opt.ceres.solver_options.max_num_iterations = 100
    if args.variant != 'original':
        opt.ceres.loss_function_type = pc.LossFunctionType.HUBER
        opt.ceres.loss_function_scale = 1.
        opt.min_track_length = 3
    opt.refine_principal_point = args.variant == 'principal'
    pc.bundle_adjustment(rec, opt)
    rec.write(out)
    by_name = {im.name.split('/')[0]: im for im in rec.images.values() if im.has_pose}
    cams = {key: rec.cameras[im.camera_id] for key, im in by_name.items()}
    rig = next(r for r in rec.rigs.values() if r.num_sensors() == 2)
    relative = rig.sensor_from_rig(cams['right_repeater'].sensor_id)
    pairs = {}
    for name, entries in saved.items():
        usable = [(xy, rec.points3D[pid].xyz) for xy, pid in entries
                  if pid in rec.points3D and rec.points3D[pid].track.length() >= 3]
        pairs[name] = (np.array([v[0] for v in usable]), np.array([v[1] for v in usable]))
    pose = pc.estimate_and_refine_absolute_pose(*pairs['left_repeater'], cams['left_repeater'])
    if pose is None:
        raise RuntimeError('Held-out left localization failed')
    predicted = relative * pose['cam_from_world']
    xy, xyz = pairs['right_repeater']
    projected = cams['right_repeater'].img_from_cam(predicted * xyz)
    err = np.linalg.norm(projected - xy, axis=1)
    report = {
        'heldout_timestamp': args.time, 'units': 'arbitrary SfM, not metres',
        'variant': args.variant,
        'training_reprojection_px': rec.compute_mean_reprojection_error(),
        'left_localization_inliers': int(pose['num_inliers']),
        'right_prediction_points': len(err),
        'right_prediction_median_px': float(np.median(err)),
        'right_prediction_p95_px': float(np.percentile(err, 95)),
        'right_prediction_max_px': float(err.max()),
        'relative_rotation': relative.rotation.matrix().tolist(),
        'relative_translation_arbitrary_units': relative.translation.tolist(),
        'cameras': {name: cam.params.tolist() for name, cam in cams.items()},
        'accepted': bool(np.median(err) <= 3 and np.percentile(err, 95) <= 8),
    }
    (out / 'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    np.savez_compressed(out / 'heldout-projections.npz', observed=xy, predicted=projected)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
