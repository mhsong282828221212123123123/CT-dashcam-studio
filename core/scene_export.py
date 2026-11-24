"""Cancellable MP4 export of an already loaded 3D timeline, without inference."""
from pathlib import Path
import subprocess,tempfile,sys
from contextlib import ExitStack
import cv2,numpy as np,imageio_ffmpeg
from core.driving_scene import DrivingRenderer
from core.scene_pipeline import AnalysisCancelled
from core.scene_analysis import CameraReader


class OriginalCameraStrip:
    names=('front','back','left_repeater','right_repeater')

    def __init__(self,manifest,directory,output):
        self.readers={}
        try:
            for name in self.names:
                meta=manifest.get('cameras',{}).get(name)
                if not meta:continue
                filename=meta['file']
                if Path(filename).name!=filename:raise ValueError('Invalid source filename')
                path=Path(directory)/filename
                if path.resolve()==Path(output).resolve():raise ValueError('원본 영상에 덮어쓸 수 없습니다.')
                if path.is_file():self.readers[name]=CameraReader(path)
            if 'front' not in self.readers:raise ValueError('원본 전방 영상이 있는 폴더를 선택하세요.')
        except Exception:self.close();raise

    def close(self):
        for reader in self.readers.values():reader.close()

    def render(self,t):
        tiles=[]
        for name in self.names:
            reader=self.readers.get(name);frame=reader.read_at(t)[1] if reader else None
            tile=np.zeros((180,240,3),np.uint8)
            if frame is not None:
                h,w=frame.shape[:2];scale=min(240/w,180/h)
                resized=cv2.resize(frame,(round(w*scale),round(h*scale)))
                y=(180-resized.shape[0])//2;x=(240-resized.shape[1])//2
                tile[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
            else:cv2.putText(tile,'MISSING',(65,95),0,.6,(0,150,255),1)
            cv2.putText(tile,name,(5,17),0,.42,(0,255,255),1,cv2.LINE_AA)
            tiles.append(tile)
        return np.hstack(tiles)


def export_scene_video(timeline,output,fps=30,progress=None,cancelled=None,overwrite=False,manifest=None,source_directory=None):
    output=Path(output)
    if output.suffix.lower()!='.mp4':raise ValueError('MP4 파일을 선택하세요.')
    if output.exists() and not overwrite:raise FileExistsError(output)
    if not timeline.scenes:raise ValueError('분석 결과가 비어 있습니다.')
    if not 1<=fps<=60:raise ValueError('Invalid export FPS')
    times=np.array([s.time for s in timeline.scenes]);step=float(np.median(np.diff(times))) if len(times)>1 else .1
    duration=times[-1]-times[0]+step;count=max(1,int(round(duration*fps)))
    renderer=DrivingRenderer(960,640);output.parent.mkdir(parents=True,exist_ok=True)
    with ExitStack() as stack:
        strip=None
        if manifest is not None or source_directory is not None:
            if manifest is None or source_directory is None:raise ValueError('Source folder and camera metadata required')
            strip=OriginalCameraStrip(manifest,source_directory,output);stack.callback(strip.close)
        directory=stack.enter_context(tempfile.TemporaryDirectory(prefix='.scene-export-',dir=output.parent))
        temporary=Path(directory)/'video.mp4'
        command=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-f','rawvideo','-pix_fmt','bgr24',
            '-s','960x820' if strip else '960x640','-r',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast',
            '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)]
        with tempfile.TemporaryFile() as errors:
            process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=errors,
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform=='win32' else 0)
            try:
                for i in range(count):
                    if cancelled and cancelled():raise AnalysisCancelled()
                    timestamp=float(times[0]+i/fps)
                    frame=renderer.render(timeline.sample(timestamp))
                    if strip:frame=np.vstack([frame,strip.render(timestamp)])
                    process.stdin.write(frame.tobytes())
                    if progress:progress(int((i+1)*95/count))
                process.stdin.close()
                while True:
                    if cancelled and cancelled():raise AnalysisCancelled()
                    try:code=process.wait(timeout=.1);break
                    except subprocess.TimeoutExpired:continue
                if code:
                    errors.seek(0);raise RuntimeError(errors.read(3000).decode('utf-8',errors='replace'))
            finally:
                if process.poll() is None:process.terminate();process.wait(timeout=10)
                if process.stdin and not process.stdin.closed:
                    try:process.stdin.close()
                    except BrokenPipeError:pass
        if cancelled and cancelled():raise AnalysisCancelled()
        if output.exists() and not overwrite:raise FileExistsError(output)
        temporary.replace(output)
    if progress:progress(100)
    return str(output)
