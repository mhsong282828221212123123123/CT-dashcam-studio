"""Reproject proposed oriented envelopes into original images for review."""
import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline,corners,dimensions
from core.scene_analysis import CameraReader

root=Path(sys.argv[1]);out=root/'trajectory-reprojection';out.mkdir(exist_ok=True)
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'));tiles=[]
for case in cases:
    report=root/'trajectory-review'/(case['id']+'.json')
    if not report.exists():continue
    proposals=json.loads(report.read_text(encoding='utf-8'))
    proposals=sorted([p for p in proposals if p['accepted'] and p['improvement']>.05],
                     key=lambda p:p['improvement'],reverse=True)[:3]
    if not proposals:continue
    target=root/'baseline'/case['id']
    rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    timeline=FusedSceneTimeline(rows,manifest)
    for proposal in proposals:
        point=proposal['positions'][len(proposal['positions'])//2]
        i=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-point['time']))
        camera=proposal['source'].split(':')[0]
        reader=CameraReader(Path(case['source']).parent/manifest['cameras'][camera]['file'])
        try:_,frame=reader.read_at(point['time'])
        finally:reader.close()
        obj=next(o for o in rows[i]['cameras'][camera]['objects'] if o['track_id']==proposal['source'])
        x,y,w,h=map(round,obj['bbox']);cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        geometry=timeline.camera_frames[i][camera]
        xyz=corners(*point['position'],dimensions('car'),yaw=point['axis_yaw'],kind='car')
        pixels=geometry.pixels(xyz)
        if pixels is not None:
            cv2.polylines(frame,[cv2.convexHull(pixels.astype(np.float32)).astype(np.int32)],True,(255,0,255),2)
            ground=pixels[np.isclose(xyz[:,1],0)]
            cv2.polylines(frame,[cv2.convexHull(ground.astype(np.float32)).astype(np.int32)],True,(255,255,0),2)
        tile=cv2.resize(frame,(640,480))
        cv2.putText(tile,case['id']+' '+proposal['source'],(5,20),0,.48,(0,255,255),1)
        cv2.putText(tile,f"yaw {np.degrees(point['axis_yaw']):.0f} / held-out IoU {proposal['validation_iou_median']:.2f}",(5,465),0,.5,(0,255,255),1)
        cv2.imencode('.jpg',tile)[1].tofile(str(out/(case['id']+'-'+proposal['source'].replace(':','-')+'.jpg')))
        tiles.append(tile)
for i in range(0,len(tiles),4):
    page=tiles[i:i+4]
    while len(page)<4:page.append(np.zeros_like(page[0]))
    cv2.imencode('.jpg',np.vstack([np.hstack(page[:2]),np.hstack(page[2:])]))[1].tofile(str(out/f'sheet-{i//4:02d}.jpg'))
print('Rendered proposals',len(tiles),flush=True)
