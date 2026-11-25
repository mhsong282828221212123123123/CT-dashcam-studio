"""Compare identical original observations; missing predictions are never successes."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import canonical


def compare(before,after):
    key=lambda r:(r['camera_id'],r['frame_id'],r['time_s'],tuple(r['source_bbox']))
    A={key(r):r for r in before['records']};B={key(r):r for r in after['records']};out={}
    for cam in sorted({k[0] for k in A.keys()|B.keys()}):
        common=[k for k in A.keys()&B.keys() if k[0]==cam]
        def stats(index):
            if not common:return None
            rows=[index[k] for k in common]
            return dict(median_height_ratio=float(np.median([r['height_ratio'] for r in rows])),median_bbox_iou=float(np.median([r['bbox_iou'] for r in rows])),p90_centre_x_error_px=float(np.percentile([r['centre_x_error_px'] for r in rows],90)),p90_abs_log_height_ratio=float(np.percentile([abs(math.log(max(r['height_ratio'],1e-9))) for r in rows],90)))
        out[cam]=dict(common_projected_observations=len(common),before=stats(A),after=stats(B),lost_projections=sum(k[0]==cam for k in A.keys()-B.keys()),gained_projections=sum(k[0]==cam for k in B.keys()-A.keys()),before_source_counts=before['summary'].get(cam),after_source_counts=after['summary'].get(cam))
    return dict(metric_accuracy='NOT_EVALUATED',comparison=out,scope='Identical source keys, estimated-rig cuboid reprojection proxy; omitted/lost/gained sources reported separately, not silently counted as improvements')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=compare(json.loads(a.before.read_text()),json.loads(a.after.read_text()));a.output.write_bytes(canonical(result));print(json.dumps(result))
