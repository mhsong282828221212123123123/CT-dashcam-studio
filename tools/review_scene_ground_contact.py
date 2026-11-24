"""Compare box contact residuals; these are NOT surveyed position errors."""
import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import core.scene_fusion as fusion


def review(directory):
    directory=Path(directory)
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    rows=[json.loads(line) for line in (directory/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    timeline=fusion.FusedSceneTimeline(rows,manifest,postprocess=False,
                                     enable_handoffs=False,select_truck_shapes=False)
    scales=fusion.box_fit_scales
    def old_scales(camera,w,h,weighted=True):
        return scales(camera,w,h,weighted=False)
    accepted=[0,0];common=[];near_delta=[];changed=[]
    for index in np.unique(np.linspace(0,len(rows)-1,min(40,len(rows))).astype(int)):
        camera=timeline.camera_frames[index].get('front')
        if camera is None:continue
        for obj in rows[index]['cameras'].get('front',{}).get('objects',[]):
            if obj.get('track_status')!='confirmed' or obj['kind'] not in ('car','truck','bus'):continue
            kind=obj.get('render_kind') or obj['kind'];box=obj['bbox']
            with patch.object(fusion,'box_fit_scales',old_scales):
                before=fusion.fit_box(camera,box,kind)
            after=fusion.fit_box(camera,box,kind)
            accepted[0]+=before is not None;accepted[1]+=after is not None
            if (before is None)!=(after is None):
                changed.append(dict(time_s=rows[index]['time_s'],id=obj['track_id'],
                                    before=before is not None,after=after is not None))
            if before is None or after is None:continue
            errors=[];ious=[]
            for fit in (before,after):
                bounds=camera.visible_bounds(fusion.corners(*fit[0],fit[1],kind=kind))
                errors.append(abs(bounds[3]-(box[1]+box[3])))
                ious.append(fusion.iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],box))
            common.append((errors,ious))
            if box[3]>=60:near_delta.append(abs(after[0][0])-abs(before[0][0]))
    return dict(result=str(directory),accepted_before_after=accepted,common_count=len(common),
                median_bottom_error_px_before_after=np.median([c[0] for c in common],axis=0).tolist() if common else [],
                median_iou_before_after=np.median([c[1] for c in common],axis=0).tolist() if common else [],
                near_median_absolute_x_delta_m=float(np.median(near_delta)) if near_delta else None,
                acceptance_changes=changed,
                limitation='Contact residual measures image agreement, not real-world position accuracy. No manual positions used.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',action='append',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    reports=[]
    for result in args.result:
        report=review(result);reports.append(report)
        print(json.dumps(report),flush=True)
    args.output.write_text(json.dumps(reports,indent=2),encoding='utf-8')
