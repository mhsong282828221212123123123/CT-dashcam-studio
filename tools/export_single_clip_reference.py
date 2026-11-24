"""Export a locally rebuilt reference and decode every output frame for QA."""
import json,pickle,sys
from pathlib import Path
import cv2,numpy as np
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_export import export_scene_video
from core.driving_scene import DrivingRenderer

if __name__=='__main__':
    root=Path(sys.argv[1]);tag=sys.argv[2]
    with (root/(tag+'.pkl')).open('rb') as f:timeline=pickle.load(f)
    manifest=json.loads((root/'reviewed-input'/'manifest.json').read_text(encoding='utf8'))
    output=root/(tag+'-4camera-35s.mp4');last=[-1]
    def progress(value):
        if value//10!=last[0]:print('Export',value,flush=True);last[0]=value//10
    if '--diagnostic' in sys.argv:
        audit=json.loads((root/(tag+'-audit.json')).read_text(encoding='utf8'))
        outliers=json.loads((root/(tag+'-motion-outliers.json')).read_text(encoding='utf8'))
        label=f"REVIEW ONLY / NOT PASSED: missing {audit['missing_frames']}/{audit['observation_frames']} | jump steps {len(outliers)}"
        class DiagnosticRenderer(DrivingRenderer):
            def render(self,scene):
                f=super().render(scene)
                cv2.rectangle(f,(12,580),(948,626),(225,235,255),-1)
                cv2.putText(f,label,(22,598),0,.48,(30,35,170),1,cv2.LINE_AA)
                cv2.putText(f,'Ghost: unverified position / road height. Original cameras below.',(22,618),0,.43,(60,60,100),1,cv2.LINE_AA)
                return f
        with patch('core.scene_export.DrivingRenderer',DiagnosticRenderer):
            export_scene_video(timeline,output,manifest=manifest,
                source_directory=Path(manifest['source_front_path']).parent,progress=progress)
    else:
        export_scene_video(timeline,output,manifest=manifest,
            source_directory=Path(manifest['source_front_path']).parent,progress=progress)
    cap=cv2.VideoCapture(str(output));fps=cap.get(cv2.CAP_PROP_FPS)
    count=0;samples=[];black=[]
    while True:
        ok,frame=cap.read()
        if not ok:break
        if frame.shape!=(820,960,3):raise ValueError(frame.shape)
        if frame.mean()<1:black.append(count)
        if count%30==0:
            tile=cv2.resize(frame,(480,410),interpolation=cv2.INTER_AREA)
            cv2.putText(tile,f'OUTPUT {count/fps:.1f}s',(180,20),0,.55,(0,0,230),2)
            samples.append(tile)
        count+=1
    cap.release()
    if count!=1050 or black:raise ValueError(dict(count=count,black=black))
    for page in range((len(samples)+8)//9):
        tiles=samples[page*9:page*9+9]
        tiles+=[np.zeros_like(tiles[0])]*(9-len(tiles))
        sheet=np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,9,3)])
        cv2.imencode('.jpg',sheet)[1].tofile(str(root/f'{tag}-qa-{page+1}.jpg'))
    report=dict(path=str(output.resolve()),frames=count,fps=fps,duration_s=count/fps,
        dimensions=[960,820],decoded_all_frames=True,black_frames=black,
        visual_review_sheets=(len(samples)+8)//9)
    (root/(tag+'-export-qa.json')).write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report),flush=True)
