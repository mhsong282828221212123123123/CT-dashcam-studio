"""CPU parity + held-out visual label review; static snapshots, not 3D replay."""
import argparse,json,hashlib
from pathlib import Path
from fractions import Fraction
import numpy as np,av
from PIL import Image,ImageDraw,ImageFont
from ct_scene.learned_contact import LearnedContact,preprocess
from ct_scene.contact_learning import contact_target
p=argparse.ArgumentParser();p.add_argument('--annotations',type=Path,required=True);p.add_argument('--features',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--reference-predictions',type=Path,default=Path('runs/contact-learning-trained-001/predictions.json'));p.add_argument('--snapshot-seconds',type=float,default=1.5);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
records=json.loads(a.annotations.read_text())['records'];features=np.load(a.features/'features.npz');model=LearnedContact(a.model);old=json.loads(a.reference_predictions.read_text());repeat=round(a.snapshot_seconds*20);assert repeat>0;font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',19);small=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',16);frames=[];parity=[];cache={};predictions=[]
for i,r in enumerate(records):
 crop=np.array(Image.open(r['crop']).convert('RGB'));prediction=model.predict(crop,r);predictions.append(dict(id=r['id'],**prediction));outs=model.encoder.run(None,{'rgb_normalized':preprocess(crop)});onnx=outs[1][0] if model.report.get('schema')=='ct_scene.experimental_contact_adapter_v4' else outs[0][0];parity.append(float(np.max(abs(onnx-features['features'][i]))))
 if np.max(abs(np.array(prediction['contact_uv'])-old[i]['contact_uv']))>1e-5 or prediction['direction']!=old[i]['direction']:raise ValueError('Packaged inference differs from training')
 if r['split']!='test':continue
 source=Path(r['source_image']);assert hashlib.sha256(source.read_bytes()).hexdigest()==r['source_image_sha256']
 im=Image.new('RGB',(1280,1000),'#191d22');original=Image.open(source).convert('RGB').resize((960,720));im.paste(original,(0,0));draw=ImageDraw.Draw(im);b=np.array(r['bbox_xyxy']);wh=b[2:]-b[:2]
 def dot(uv,color,radius):
  x,y=(b[:2]+np.array(uv)*wh)*.75;draw.ellipse((x-radius,y-radius,x+radius,y+radius),outline=color,width=3)
 dot([.5,1],'#ff7777',8);dot(prediction['contact_uv'],'#45cfff',6)
 target=contact_target(r)
 if target is not None:dot(target,'#60ef8a',10)
 x,y,X,Y=b*.75;draw.rectangle((x,y,X,Y),outline='#f4c756',width=1)
 draw.rectangle((0,0,960,64),fill='#171b20');draw.text((15,9),'학습에 쓰지 않은 원본 정지 프레임 · 3D 재현 영상 아님',font=font,fill='white');draw.text((15,36),r['id']+' / '+r['camera'],font=small,fill='white')
 for j,(label,color) in enumerate([('기존 상자 바닥점','#ff7777'),('학습한 접점 후보','#45cfff'),('직접 검토한 이미지 라벨','#60ef8a')]):draw.text((975,20+32*j),label,font=small,fill=color)
 draw.text((975,140),'방향 라벨: '+str(r['direction']),font=small,fill='white');draw.text((975,170),'모델 후보: '+prediction['direction'],font=small,fill='white');draw.text((975,610),'실험용 · 기본 기능 미적용',font=small,fill='#f4c756');draw.text((975,642),'거리·3D 각도 정답 없음',font=small,fill='#bbbbbb')
 patch=Image.fromarray(crop);patch.thumbnail((310,360));im.paste(patch,(965,230))
 package=source.parent.parent
 frames_json=package/'frames.json'
 if frames_json.exists():
  if str(package) not in cache:cache[str(package)]=json.loads(frames_json.read_text())
  sourceframes=cache[str(package)];f=next(x for x in sourceframes if x['frame_id']==r['frame_id']);t=f['camera_time_s']
  for col,camera in enumerate(['front','back','left_repeater','right_repeater']):
   rows=[x for x in sourceframes if x['camera_id']==camera]
   if not rows:continue
   q=min(rows,key=lambda x:abs(x['camera_time_s']-t));tile=Image.open(package/q['image']).convert('RGB').resize((320,240));im.paste(tile,(col*320,760));draw.text((col*320+4,740),camera+' '+str(round(q['camera_time_s'],2))+'s',font=small,fill='white')
 else:
  draw.text((15,820),'E:/tesla 직접 추출 원본 정지 프레임 (4채널 패키지 미생성)',font=small,fill='#bbbbbb')
 frames.append(im)
video=a.output/'heldout-contact-review.mp4'
with av.open(str(video),'w',options={'movflags':'+faststart'}) as out:
 stream=out.add_stream('libx264',rate=20);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'};i=0
 for im in frames:
  for _ in range(repeat):
   f=av.VideoFrame.from_image(im);f.pts=i;f.time_base=Fraction(1,20);i+=1
   for packet in stream.encode(f):out.mux(packet)
 for packet in stream.encode():out.mux(packet)
with av.open(str(video)) as c:decoded=sum(1 for _ in c.decode(video=0))
assert decoded==len(frames)*repeat
sheet=Image.new('RGB',(1280,1000))
for j,k in enumerate(np.linspace(0,len(frames)-1,4).round().astype(int)):sheet.paste(frames[k].resize((640,500)),((j%2)*640,(j//2)*500))
sheet.save(a.output/'heldout-sheet.jpg')
(a.output/'review.json').write_text(json.dumps(dict(cpu_provider=model.encoder.get_providers(),max_torch_onnx_feature_abs_difference=max(parity),all_packaged_predictions_match_training=True,heldout_source_snapshots=len(frames),frames_decoded=decoded,retained_four_source_cameras=True,runtime_adopted=False,scope='Static original-image label/prediction review, not a new 3D result or video tracking validation'),indent=2));(a.output/'predictions.json').write_text(json.dumps(predictions,indent=2));print('CPU parity',max(parity),'decoded',decoded)
