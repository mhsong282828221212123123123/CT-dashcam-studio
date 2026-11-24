"""Raw-reviewed third-lane black sedan: front -> right repeater -> clip end."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_bounds,supported
from review_guards import require_window


def footprint_review(states,key,road,length,width,begin,end,lane):
    require_window(states,begin,end);missing=[];outside=[];worst=0.;count=0
    for f in states:
        if not begin<=f['time_s']<=end:continue
        count+=1;a=next((a for a in f['actors'] if a['id']==key),None)
        if not a:missing.append(f['time_s']);continue
        yaw=a['yaw_rad'];R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        corners=np.array([[x*length/2,y*width/2] for x,y in [(-1,-1),(-1,1),(1,-1),(1,1)]])@R.T+np.asarray(a['position_W'])[:2]
        for s,d,_ in coordinates(road,corners):
            if not supported(road,float(s)):outside.append(dict(time_s=f['time_s'],reason='unsupported_road'));break
            upper,lower=lane_bounds(road,float(s),lane);violation=max(float(d-upper),float(lower-d),0.)
            worst=max(worst,violation)
            if violation>.05:outside.append(dict(time_s=f['time_s'],outside_m=violation));break
    return dict(gate='PASS' if count and not missing and not outside else 'FAIL',frames=count,missing_frames=missing,
                outside_body_events=outside,worst_body_outside_m=worst,required_interval_s=[begin,end],expected_lane=lane)


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    manifest=verify(args.scene)
    assert any(r['role']=='source' and r['artifact_id']=='66941f33999fa1733fb0624bc0a55be256379e446313eed9e3e0dfa489f0b498' for r in manifest['inputs'])
    actors=json.loads((args.scene/'actors.json').read_text());actor=next((a for a in actors if {'front-0003','right_repeater-0003'}<=set(a['local_ids'])),None)
    states=[json.loads(l) for l in (args.scene/'render-states.jsonl').read_text().splitlines()];report=json.loads((args.scene/'report.json').read_text());length,width=report['vehicle_dimensions_m']['sedan'][:2]
    result=footprint_review(states,actor['global_id'] if actor else '__missing__',report['road'],length,width,49.,states[-1]['time_s'],2)
    result.update(same_source_identity=bool(actor),source_evidence='runs/oct02-right-tail-source-001.jpg',scope='Selected black sedan remains fully inside original third lane through full export end; not all actors/source accuracy')
    if not actor:result['gate']='FAIL'
    args.output.mkdir(exist_ok=False,parents=True);(args.output/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k not in ('outside_body_events','missing_frames')}));return 0 if result['gate']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
