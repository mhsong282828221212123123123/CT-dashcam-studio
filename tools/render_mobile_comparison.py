"""Render a portrait MP4 with reconstruction and synchronized original cameras."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image,ImageDraw,ImageFont
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.driving_scene import DrivingRenderer
from core.scene_fusion import FusedSceneTimeline
from core.scene_analysis import CameraReader


def render(result,source,output,duration=30.,fps=15,start_offset=0.,title=None,subtitle=None,timeline=None,footer_lines=None):
    root,source,output=map(Path,(result,source,output))
    if output.exists():raise FileExistsError(output)
    rows=[json.loads(x) for x in (root/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    start=rows[0]['time_s']+start_offset
    if start+duration>rows[-1]['time_s']+1/manifest['sample_hz']+.001:raise ValueError('Insufficient analyzed duration')
    if timeline is None:timeline=FusedSceneTimeline(rows,manifest)
    renderer=DrivingRenderer(720,480)
    names=('front','back','left_repeater','right_repeater')
    readers={name:CameraReader(source/manifest['cameras'][name]['file']) for name in names}
    plate=Image.new('RGB',(720,1280),(22,27,33));draw=ImageDraw.Draw(plate)
    font=lambda size:ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',size)
    draw.text((20,8),title or f'{duration:.0f}초 주행 재현 · 원본 비교',font=font(28),fill='white')
    draw.text((20,46),subtitle or '전방·측방 중심  |  후방은 확인용 보조',font=font(21),fill=(190,210,225))
    draw.text((20,568),'원본 4채널 · 같은 재생 시각',font=font(23),fill='white')
    footer=footer_lines or ['반투명 차량 = 추정 중 · 자동 분석',
        '차선: 카메라 관측 · OSM: 도로 형태 참고',
        '검토용 샘플 · 세부 차선 재구성 / 지도 영상 정렬 미완성']
    for text,y,size,color in zip(footer,(1160,1204,1240),(24,21,18),('white',(200,210,220),(200,190,170))):
        draw.text((20,y),text,font=font(size),fill=color)
    plate=cv2.cvtColor(np.array(plate),cv2.COLOR_RGB2BGR)
    output.parent.mkdir(parents=True,exist_ok=True)
    temp=output.with_name(output.stem+'.partial.mp4')
    if temp.exists():raise FileExistsError(temp)
    command=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-f','rawvideo','-pix_fmt','bgr24','-s','720x1280',
             '-r',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','23',
             '-pix_fmt','yuv420p','-movflags','+faststart',str(temp)]
    with tempfile.TemporaryFile() as errors:
        process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=errors,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            for i in range(round(duration*fps)):
                t=start+i/fps;frame=plate.copy()
                frame[80:560]=renderer.render(timeline.sample(min(t,rows[-1]['time_s'])))
                for k,name in enumerate(names):
                    _,original=readers[name].read_at(t)
                    if original is None:raise ValueError(f'Missing source frame: {name} {t}')
                    tile=cv2.resize(original,(360,270),interpolation=cv2.INTER_AREA)
                    cv2.rectangle(tile,(0,0),(360,27),(15,20,25),-1)
                    cv2.putText(tile,name.upper(),(8,19),0,.45,(255,255,255),1,cv2.LINE_AA)
                    x=(k%2)*360;y=600+(k//2)*270;frame[y:y+270,x:x+360]=tile
                cv2.putText(frame,f'{i/fps:04.1f} / {duration:.0f}s',(520,590),0,.55,(255,255,255),1,cv2.LINE_AA)
                process.stdin.write(frame.tobytes())
                if i in (0,round(12*fps),round(22*fps)):
                    cv2.imencode('.jpg',frame)[1].tofile(str(output.with_name(f'{output.stem}-preview-{i:04d}.jpg')))
                if i%150==0:print(f'Rendered {i}/{round(duration*fps)}',flush=True)
            process.stdin.close();code=process.wait(timeout=120)
            if code:
                errors.seek(0);raise RuntimeError(errors.read().decode(errors='replace'))
            temp.rename(output)
        finally:
            if process.poll() is None:process.kill();process.wait()
            for reader in readers.values():reader.close()
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('result','source','output'):p.add_argument('--'+key,required=True)
    p.add_argument('--duration',type=float,default=30);p.add_argument('--fps',type=int,default=20)
    p.add_argument('--start-offset',type=float,default=0);p.add_argument('--title');p.add_argument('--subtitle')
    a=p.parse_args();print(render(a.result,a.source,a.output,a.duration,a.fps,a.start_offset,a.title,a.subtitle))
