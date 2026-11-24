"""Raw-only heldout anchors and ego-event sheets; no prediction file is read."""
import json
from pathlib import Path
import av
from PIL import Image,ImageDraw
from ct_scene.ingest import file_hash


def raw_frames(path,times):
    pending=iter(sorted(times));target=next(pending,None);previous=None
    with av.open(str(path)) as c:
        for f in c.decode(video=0):
            if f.pts is None:continue
            now=float(f.pts*f.time_base)
            while target is not None and now>=target:
                nearest=previous if previous is not None and abs(float(previous.pts*previous.time_base)-target)<abs(now-target) else f
                yield target,nearest.to_image(),dict(pts=nearest.pts,time_base=str(nearest.time_base),time_s=float(nearest.pts*nearest.time_base))
                target=next(pending,None)
            if target is None:break
            previous=f


selections=[json.loads(Path(p).read_text()) for p in ['runs/new-date-selection-003/selection.json','runs/new-date-selection-005/selection.json']]
sources={s['source_id']:s for m in selections for s in m['selected']}
specs=[('white_pickup','2026-08-20_17-27-11',[20,40]),('dark_suv_aug28','2026-08-28_08-55-51',[10,20]),
       ('dark_suv_sep09','2026-09-09_18-11-13',[10,40]),('bus_11','2026-08-26_08-32-43',[10,40]),('black_taxi','2026-10-22_08-11-29',[0,10])]
out=Path('runs/new-holdout-raw-anchors-001');out.mkdir(exist_ok=False);manifest=[]
for name,sid,times in specs:
    s=sources[sid];path=s['source_files']['front'];assert file_hash(Path(path))==s['source_sha256']['front']
    for request,im,frame in raw_frames(path,times):
        file=f'{name}-{request:04.1f}.png';im.save(out/file)
        manifest.append(dict(object_id=name,source_id=sid,camera='front',source_sha256=s['source_sha256']['front'],request_s=request,file=file,**frame))
for name,sid,times in [('aug20-left-candidate','2026-08-20_17-27-11',range(0,14)),('aug28-left-candidate','2026-08-28_08-55-51',range(18,34))]:
    canvas=Image.new('RGB',(4*480,4*380));draw=ImageDraw.Draw(canvas)
    for i,(request,im,frame) in enumerate(raw_frames(sources[sid]['source_files']['front'],times)):
        im.thumbnail((480,360));x=i%4*480;y=i//4*380;canvas.paste(im,(x,y+20));draw.text((x+3,y+2),str(round(frame['time_s'],3)),fill='white')
    canvas.save(out/(name+'.jpg'))
(out/'frames.json').write_text(json.dumps(dict(predictions_used=False,anchors=manifest),indent=2))
print('Wrote ten raw anchors and two dense ego-maneuver sheets; manual review pending.')
