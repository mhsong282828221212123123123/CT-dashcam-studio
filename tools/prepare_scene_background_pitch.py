"""Create a source-bound reviewed pitch trace; original videos stay read-only."""
import argparse,json,hashlib,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader
from core.scene_background_pitch import reference_points,estimate_pitch
from core.scene_fusion import cameras_for,load_profile
from core.scene_stopped_motion import stopped_motion_priors


def prepare(result,source,output,reference_time):
    result,source,output=Path(result),Path(source),Path(output)
    if output.exists():raise FileExistsError(output)
    raw=(result/'observations.jsonl').read_bytes();rows=[json.loads(l) for l in raw.decode('utf-8').splitlines()]
    manifest=json.loads((result/'manifest.json').read_text());base=cameras_for(manifest,load_profile(manifest))['front']
    horizons,_,reports=stopped_motion_priors(rows,base)
    index=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-reference_time))
    if index not in horizons:raise ValueError('Reference must have supported stopped calibration')
    cap=cv2.VideoCapture(str(source));cap.set(cv2.CAP_PROP_POS_MSEC,reference_time*1000);ok,frame=cap.read();cap.release()
    if not ok:raise ValueError('Reference frame unavailable')
    reference=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);points=reference_points(reference);trace=[];reader=CameraReader(source)
    try:
        for row in rows:
            _,frame=reader.read_at(row['time_s'])
            item=estimate_pitch(reference,cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),points,horizons[index]) if frame is not None else None
            trace.append(dict(time_s=row['time_s'],**item) if item else dict(time_s=row['time_s'],status='unsupported'))
    finally:reader.close()
    valid=[i for i,item in enumerate(trace) if 'horizon_px' in item]
    if not valid or any(min(abs(rows[i]['time_s']-rows[j]['time_s']) for j in valid)>.5 for i in range(len(rows))):
        raise ValueError('Background support missing for more than .5 seconds')
    values=np.interp(np.arange(len(trace)),valid,[trace[i]['horizon_px'] for i in valid])
    values=np.convolve(np.pad(values,2,mode='edge'),np.array([1,2,3,2,1])/9,mode='valid')
    for item,value in zip(trace,values):item['horizon_px']=float(value)
    payload=dict(front_file=source.name,reference_time_s=reference_time,reference_horizon_px=horizons[index],
        source='static_upper_background_forward_backward_flow_and_affine_registration',
        observation_sha256=hashlib.sha256(raw).hexdigest(),width=base.width,height=base.height,
        reference_calibration=reports[index],trace=trace,status='estimated_on_this_clip',surveyed=False)
    output.write_text(json.dumps(payload,indent=2),encoding='utf-8')
    print('background pitch',len(valid),'/',len(trace),'validated frames; range',round(float(values.min()),2),round(float(values.max()),2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('result','source','output'):p.add_argument('--'+key,required=True)
    p.add_argument('--reference-time',type=float,required=True)
    a=p.parse_args();prepare(a.result,a.source,a.output,a.reference_time)
