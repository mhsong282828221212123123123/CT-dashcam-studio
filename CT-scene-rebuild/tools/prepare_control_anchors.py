"""Raw PTS anchors for independent object identity/relationship controls."""
import json
from pathlib import Path
import av
from ct_scene.artifacts import canonical

out=Path('runs/source-control-anchors-001');out.mkdir(exist_ok=False)
sources={s['source_id']:s for file in ['runs/new-date-selection-001/selection.json','runs/new-date-selection-002/selection.json'] for s in json.loads(Path(file).read_text())['selected']}
requests=[('2026-09-08_17-55-47','front',30.),('2026-09-08_17-55-47','front',39.5),
          ('2026-10-26_12-01-20','front',20.),('2026-10-26_12-01-20','front',50.),
          ('2026-09-20_21-59-02','front',10.),('2026-09-20_21-59-02','front',50.),
          ('2026-09-14_22-06-27','left_repeater',30.),('2026-09-14_22-06-27','left_repeater',50.)]
records=[]
for source_id,camera,t in requests:
    source=sources[source_id]
    with av.open(source['source_files'][camera]) as c:
        stream=c.streams.video[0];c.seek(int(t/stream.time_base),stream=stream,backward=True);previous=None
        for frame in c.decode(stream):
            now=float(frame.pts*frame.time_base)
            if now<t:previous=frame;continue
            if previous is not None and abs(float(previous.pts*previous.time_base)-t)<=abs(now-t):frame=previous
            name=f'{source_id}-{camera}-{t:.1f}.png';frame.to_image().save(out/name)
            records.append(dict(source_id=source_id,camera=camera,requested_time_s=t,time_s=float(frame.pts*frame.time_base),pts=frame.pts,time_base=str(frame.time_base),source_sha256=source['source_sha256'][camera],image=name))
            break
(out/'frames.json').write_bytes(canonical(records));print('Raw anchors:',len(records))
