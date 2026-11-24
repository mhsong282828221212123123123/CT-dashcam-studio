"""Join review excerpts while retaining the complete 3D + four-camera frame."""
import argparse,json
from pathlib import Path
from fractions import Fraction
import av
from PIL import ImageDraw,ImageFont

p=argparse.ArgumentParser()
p.add_argument('--clips',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
clips=json.loads(a.clips.read_text(encoding='utf-8'));font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',21)
expected=0;count=0
with av.open(str(a.output),'w',options={'movflags':'+faststart'}) as out:
 stream=out.add_stream('libx264',rate=20);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
 for clip in clips:
  start=round(clip['start_s']*20);end=round(clip['end_s']*20)
  if not 0<=start<end:raise ValueError('Invalid excerpt interval')
  expected+=end-start
  with av.open(str(clip['video'])) as c:
   if abs(float(c.streams.video[0].average_rate)-20)>.01:raise ValueError('Expected current 20fps full scene')
   for i,f in enumerate(c.decode(video=0)):
    if i<start:continue
    if i>=end:break
    im=f.to_image();d=ImageDraw.Draw(im);d.rectangle((12,670,1255,710),fill=(25,29,34));d.text((23,675),clip['label'],font=font,fill=(225,232,240))
    frame=av.VideoFrame.from_image(im);frame.pts=count;frame.time_base=Fraction(1,20)
    for packet in stream.encode(frame):out.mux(packet)
    count+=1
 for packet in stream.encode():out.mux(packet)
if count!=expected:raise ValueError('Source interval incomplete')
with av.open(str(a.output)) as c:
 decoded=sum(1 for _ in c.decode(video=0))
if decoded!=expected:raise ValueError('Incomplete excerpt export')
a.output.with_suffix('.review.json').write_text(json.dumps(dict(clips=clips,frames=count,decoded_frames=decoded,retained_four_cameras=True),ensure_ascii=False,indent=2),encoding='utf-8')
print('Review excerpt decoded:',decoded,'frames')
