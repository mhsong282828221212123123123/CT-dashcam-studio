"""Source-only 5 Hz object visibility sheets, with manually chosen raw ROIs."""
import argparse,json,math
from pathlib import Path
import av,numpy as np
from PIL import Image,ImageDraw,ImageFont
from ct_scene.artifacts import canonical

parser=argparse.ArgumentParser();parser.add_argument('--targets');parser.add_argument('--output',default='runs/dense-source-controls-001');parser.add_argument('--selections',nargs='+',default=['runs/new-date-selection-001/selection.json','runs/new-date-selection-002/selection.json']);args=parser.parse_args()
out=Path(args.output);out.mkdir(exist_ok=False)
sources={s['source_id']:s for file in args.selections for s in json.loads(Path(file).read_text())['selected']}
targets=[dict(source_id='2026-09-08_17-55-47',object_id='brown_compact',camera='front',interval_s=[30.,39.8],roi_xyxy=[420,380,1110,940],expected_lane=0,expected_lane_count=1,
              anchors=[dict(time_s=30.,bbox_xyxy=[565,430,935,765]),dict(time_s=39.5,bbox_xyxy=[445,400,1070,935])]),
         dict(source_id='2026-10-26_12-01-20',object_id='white_taxi',camera='front',interval_s=[20.,58.5],roi_xyxy=[360,350,900,830],expected_lane=0,
              anchors=[dict(time_s=20.,bbox_xyxy=[425,385,815,735]),dict(time_s=50.,bbox_xyxy=[380,380,860,790])]),
         dict(source_id='2026-09-20_21-59-02',object_id='green_ioniq5',camera='front',interval_s=[10.,50.],roi_xyxy=[10,350,980,940],expected_lane=0,expected_lane_count=1,
              anchors=[dict(time_s=10.,bbox_xyxy=[305,380,925,850]),dict(time_s=50.,bbox_xyxy=[35,385,710,900])]),
         dict(source_id='2026-09-14_22-06-27',object_id='white_bmw',camera='left_repeater',interval_s=[30.,50.],roi_xyxy=[230,320,1150,830],
              anchors=[dict(time_s=30.,bbox_xyxy=[265,345,1115,795]),dict(time_s=50.,bbox_xyxy=[265,345,1115,795])])]
if args.targets:targets=json.loads(Path(args.targets).read_text())['targets']
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',12)
for target in targets:
    source=sources[target['source_id']];start,end=target['interval_s'];requests=np.arange(start,end+.0001,.2);tiles=[];frame_rows=[]
    with av.open(source['source_files'][target['camera']]) as c:
        # Sequential decode also handles valid fragmented clips without a seek index.
        stream=c.streams.video[0];i=0;previous=None
        for frame in c.decode(stream):
            if frame.pts is None:continue
            t=float(frame.pts*frame.time_base)
            while i<len(requests) and t>=requests[i]:
                chosen=previous if previous is not None and abs(float(previous.pts*previous.time_base)-requests[i])<=abs(t-requests[i]) else frame
                im=chosen.to_image().crop(target['roi_xyxy']);im.thumbnail((120,100));tile=Image.new('RGB',(120,120),'#171b20');tile.paste(im,((120-im.width)//2,18));ImageDraw.Draw(tile).text((2,2),f'{float(requests[i]):.1f}s',font=font,fill='white');tiles.append(tile)
                frame_rows.append(dict(pts=chosen.pts,time_base=str(chosen.time_base),time_s=float(chosen.pts*chosen.time_base)));i+=1
            previous=frame
            if i>=len(requests):break
    if len(tiles)!=len(requests):raise ValueError('Incomplete source control sampling')
    sheets=[]
    for page,offset in enumerate(range(0,len(tiles),100)):
        sheet=Image.new('RGB',(1200,1200),'#171b20')
        for j,tile in enumerate(tiles[offset:offset+100]):sheet.paste(tile,((j%10)*120,(j//10)*120))
        name=f'{target["object_id"]}-{page}.jpg';sheet.save(out/name,quality=94);sheets.append(name)
    target.update(source_sha256=source['source_sha256'][target['camera']],source_frame_samples=frame_rows,contact_sheets=sheets,
                  dense_visibility_reviewed=False,anchors_reviewed=True,production_predictions_used=False)
(out/'objects.json').write_bytes(canonical(dict(scope='selected critical objects only; not exhaustive object recall',targets=targets)))
print(json.dumps([(t['object_id'],len(t['source_frame_samples']),t['contact_sheets']) for t in targets]))
