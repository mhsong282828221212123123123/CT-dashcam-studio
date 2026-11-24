"""Reproject experimental placements onto independent raw camera frames."""
import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline,corners,dimensions
from core.scene_analysis import CameraReader

root=Path(sys.argv[1]);out=root/'camera-prior-review';out.mkdir(exist_ok=True)
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
profile=json.loads((root/'experimental-camera-prior.json').read_text(encoding='utf-8'))
reports=[];tiles=[]
for case in cases:
    target=root/'baseline'/case['id']
    if not (target/'manifest.json').exists():continue
    rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    timeline=FusedSceneTimeline(rows,manifest,profile=profile)
    i=len(rows)//2;row=rows[i];scene=timeline.scenes[i]
    for camera in ('left_repeater','right_repeater'):
        if camera not in manifest['cameras']:continue
        reader=CameraReader(Path(case['source']).parent/manifest['cameras'][camera]['file'])
        try:_,frame=reader.read_at(row['time_s'])
        finally:reader.close()
        if frame is None:continue
        visible={s:a for a in scene.actors for s in a.sources}
        for obj in row['cameras'][camera]['objects']:
            if obj.get('track_status')!='confirmed':continue
            x,y,w,h=map(round,obj['bbox']);actor=visible.get(obj['track_id'])
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0) if actor else (0,0,255),2)
            cv2.putText(frame,obj['track_id'],(x,max(30,y-8)),0,.5,(255,255,255),1)
            if actor:
                shape=actor.geometry_kind or actor.kind
                bounds=timeline.camera_frames[i][camera].visible_bounds(corners(actor.x,actor.z,dimensions(shape),kind=shape))
                if bounds is not None:
                    x1,y1,x2,y2=map(round,bounds);cv2.rectangle(frame,(x1,y1),(x2,y2),(255,0,255),2)
                reports.append(dict(clip=case['id'],split=case['split'],camera=camera,time_s=row['time_s'],
                                    source=obj['track_id'],position=[actor.x,actor.z]))
        tile=cv2.resize(frame,(640,480))
        cv2.putText(tile,f"{case['id']} {camera}",(8,20),0,.5,(0,255,255),1)
        cv2.putText(tile,'GREEN detection / MAGENTA projection / RED unplaced',(8,466),0,.42,(0,255,255),1)
        cv2.imencode('.jpg',tile)[1].tofile(str(out/f"{case['id']}-{camera}.jpg"))
        tiles.append(tile)
for i in range(0,len(tiles),4):
    page=tiles[i:i+4]
    while len(page)<4:page.append(np.zeros_like(page[0]))
    sheet=np.vstack([np.hstack(page[:2]),np.hstack(page[2:])])
    cv2.imencode('.jpg',sheet)[1].tofile(str(out/f'sheet-{i//4:02d}.jpg'))
(out/'positions.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
print('Reprojected',len(reports),'placements',flush=True)
