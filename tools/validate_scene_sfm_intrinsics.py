"""Compare lens models on held-out images and held-out localization points."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import cv2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root', type=Path)
    args = ap.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scene-runtime-calibration'))
    import pycolmap as pc

    results = []
    proof = []
    for variant in ('estimated_radial', 'legacy_850_pinhole'):
        rec = pc.Reconstruction(args.root / 'sparse' / '0')
        held = {}
        for im in list(rec.images.values()):
            if im.name.endswith(('/004.0.jpg', '/012.0.jpg', '/020.0.jpg')):
                held[im.name] = [(np.array(pt.xy), pt.point3D_id) for pt in im.points2D if pt.has_point3D()]
                rec.deregister_frame(im.frame_id)
        camera = rec.cameras[1]
        if variant == 'legacy_850_pinhole':
            camera.params = np.array([850., 640., 480., 0.])
        opt = pc.BundleAdjustmentOptions()
        opt.print_summary = False
        opt.refine_focal_length = variant != 'legacy_850_pinhole'
        opt.refine_extra_params = variant != 'legacy_850_pinhole'
        opt.ceres.solver_options.max_num_iterations = 100
        pc.bundle_adjustment(rec, opt)
        training_error = rec.compute_mean_reprojection_error()
        if not np.isfinite(training_error) or training_error > 100:
            # A failed optimization is not a numerical accuracy comparison.
            results.append(dict(variant=variant, status='optimization_failed',
                                comparable=False, cases=[]))
            continue
        cases = []
        for name, entries in held.items():
            entries = [(xy, pid) for xy, pid in entries if pid in rec.points3D and rec.points3D[pid].track.length() >= 3]
            # Deterministic point-ID split: test pixels never enter pose fitting.
            train = [(xy, rec.points3D[pid].xyz) for xy, pid in entries if pid % 3 != 0]
            test = [(xy, rec.points3D[pid].xyz) for xy, pid in entries if pid % 3 == 0]
            xy, xyz = map(np.array, zip(*train))
            pose = pc.estimate_and_refine_absolute_pose(xy, xyz, camera)
            if pose is None:
                cases.append(dict(image=name, accepted=False, reason='localization failed'))
                continue
            xy, xyz = map(np.array, zip(*test))
            predicted = camera.img_from_cam(pose['cam_from_world'] * xyz)
            err = np.linalg.norm(xy-predicted, axis=1)
            if variant == 'estimated_radial':
                image = cv2.imdecode(np.fromfile(args.root / 'images' / name, np.uint8), cv2.IMREAD_COLOR)
                for observed, pred in zip(xy, predicted):
                    if not np.isfinite(pred).all():
                        continue
                    a, b = tuple(np.rint(observed).astype(int)), tuple(np.rint(pred).astype(int))
                    cv2.circle(image, a, 5, (0, 220, 0), 1, cv2.LINE_AA)
                    cv2.drawMarker(image, b, (0, 100, 255), cv2.MARKER_CROSS, 7, 1)
                tile = cv2.resize(image, (640, 480))
                cv2.rectangle(tile, (0, 0), (640, 54), (25, 25, 25), -1)
                cv2.putText(tile, name, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(tile, f'UNSEEN points: {len(err)} | p95 {np.percentile(err,95):.2f} px',
                            (12, 44), cv2.FONT_HERSHEY_SIMPLEX, .5, (180,240,180), 1, cv2.LINE_AA)
                proof.append(tile)
            cases.append(dict(image=name, test_points=len(err), median_px=float(np.median(err)),
                              p95_px=float(np.percentile(err, 95)), accepted=bool(np.percentile(err, 95) <= 3)))
        results.append(dict(variant=variant, params=camera.params.tolist(),
                            training_error_px=training_error, cases=cases))
    path = args.root / 'front-heldout-validation.json'
    path.write_text(json.dumps(results, indent=2, allow_nan=False), encoding='utf-8')
    if proof:
        banner = np.zeros((70,640,3),np.uint8)
        cv2.putText(banner,'OPTICAL REPROJECTION ONLY - NOT METRIC RIG CALIBRATION',
                    (8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(banner,'Green circle: observed | orange cross: predicted',
                    (8,48),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
        cv2.imencode('.jpg',np.vstack([banner,*proof]))[1].tofile(args.root / 'front-heldout-proof.jpg')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
