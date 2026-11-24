"""Review longitudinal paint extraction separately from road extent."""
import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_road_evidence import segmented_curves,road_cross_sections

root=Path(sys.argv[1]);out=root/'road-component-review';out.mkdir(exist_ok=True)
cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
reports=[];tiles=[]
for case in cases:
    masks=root/'road-segmentation-review'/(case['id']+'.npz')
    if not masks.exists():continue
    mask=np.load(masks)
    rows=[json.loads(s) for s in (root/'baseline'/case['id']/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    row=min(rows,key=lambda r:abs(r['time_s']-case['review_time_s']))
    curves=segmented_curves(mask['lane'],mask['road'],row['cameras']['front']['objects'])
    sections=road_cross_sections(mask['road'])
    cap=cv2.VideoCapture(case['source']);cap.set(cv2.CAP_PROP_POS_MSEC,case['review_time_s']*1000)
    ok,frame=cap.read();cap.release()
    if not ok:continue
    for line in curves:
        cv2.polylines(frame,[np.round(line['points']).astype(np.int32)],False,(0,255,255),4)
    for section in sections:
        for point in section:cv2.circle(frame,tuple(map(round,point)),3,(255,0,255),-1)
    tile=cv2.resize(frame,(640,480));cv2.putText(tile,case['id'],(5,25),0,.55,(0,255,255),1)
    tiles.append(tile)
    cv2.imencode('.jpg',tile)[1].tofile(str(out/(case['id']+'.jpg')))
    reports.append(dict(id=case['id'],curves=curves,road_sections=sections))
for i in range(0,len(tiles),4):
    page=tiles[i:i+4]
    while len(page)<4:page.append(np.zeros_like(page[0]))
    cv2.imencode('.jpg',np.vstack([np.hstack(page[:2]),np.hstack(page[2:])]))[1].tofile(str(out/f'sheet-{i//4}.jpg'))
(out/'evidence.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
print([(r['id'],len(r['curves'])) for r in reports])
