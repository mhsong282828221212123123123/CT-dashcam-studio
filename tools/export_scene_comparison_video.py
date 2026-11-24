"""Export current reconstruction and four synchronized original camera views."""
import argparse,json,os,sys,subprocess
from pathlib import Path
import cv2,numpy as np,imageio_ffmpeg
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
os.environ['QT_QPA_PLATFORM']='offscreen'
from core.typography import configure_font_backend
configure_font_backend()
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFontDatabase
from core.driving_scene import DrivingRenderer
from core.scene_fusion import FusedSceneTimeline
from core.scene_analysis import CameraReader

def main():
    p=argparse.ArgumentParser();p.add_argument('--result',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fps',type=int,default=10)
    a=p.parse_args()
    if not 1<=a.fps<=60:raise ValueError('FPS must be between 1 and 60')
    if a.output.exists():raise FileExistsError(a.output)
    app=QApplication([]);QFontDatabase.addApplicationFont('C:/Windows/Fonts/malgun.ttf')
    rows=[json.loads(s) for s in (a.result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((a.result/'manifest.json').read_text(encoding='utf-8'))
    print('Constructing current timeline',flush=True)
    timeline=FusedSceneTimeline(rows,manifest)
    print('Timeline ready',flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.with_suffix('.road-audit.json').write_text(json.dumps([
        dict(time_s=s.time,road_map=audit.get('road_map',{}),camera_junction=audit.get('camera_junction',{}),**audit.get('travel_corridors',{}))
        for s,audit in zip(timeline.scenes,timeline.audit)],indent=2),encoding='utf-8')
    renderer=DrivingRenderer(1280,720)
    names=('left_repeater','front','right_repeater','back')
    readers={name:CameraReader(a.source/manifest['cameras'][name]['file']) for name in names}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    command=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-f','rawvideo','-pix_fmt','bgr24',
        '-s','1280x960','-r',str(a.fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast',
        '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE)
    try:
        duration=rows[-1]['time_s']-rows[0]['time_s']+float(np.median(np.diff([r['time_s'] for r in rows])))
        expected_count=int(round(duration*a.fps))
        for i in range(expected_count):
            scene=timeline.sample(rows[0]['time_s']+i/a.fps)
            tiles=[]
            for name in names:
                _,original=readers[name].read_at(scene.time)
                if original is None:raise ValueError(f'Missing source frame: {name} {scene.time}')
                tile=cv2.resize(original,(320,240))
                cv2.putText(tile,name,(6,20),0,.55,(0,255,255),1,cv2.LINE_AA);tiles.append(tile)
            rendered=renderer.render(scene)
            cv2.putText(rendered,'WORK IN PROGRESS / NOT ACCEPTED',(20,690),0,.55,(30,30,170),1,cv2.LINE_AA)
            board=np.vstack([rendered,np.hstack(tiles)])
            process.stdin.write(board.tobytes())
            if i in (0,expected_count//2,expected_count-1):
                cv2.imencode('.jpg',board)[1].tofile(str(a.output.with_name(a.output.stem+f'-{i:03d}.jpg')))
            if i%100==0:print(f'Rendered {i}/{expected_count}',flush=True)
        process.stdin.close()
        if process.wait(timeout=120):raise RuntimeError('Video encoder failed')
    finally:
        for reader in readers.values():reader.close()
        if process.poll() is None:process.terminate();process.wait(timeout=10)
    cap=cv2.VideoCapture(str(a.output));count=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        if frame.shape[:2]!=(960,1280):raise ValueError('Unexpected frame size')
        count+=1
    fps=cap.get(cv2.CAP_PROP_FPS);cap.release()
    if count!=expected_count or count/fps<30:raise ValueError('Export duration/frame count failed')
    report=dict(frames=count,fps=fps,duration_s=count/fps,source=str(a.source),
                result=str(a.result),quality_accepted=False,originals_modified=False)
    a.output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
