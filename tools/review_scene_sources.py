"""Review first/middle/last source crops for every size-eligible camera track."""
import json,sys
from pathlib import Path
from collections import defaultdict
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader

def review(root,output):
    root,output=Path(root),Path(output);output.mkdir(exist_ok=True,parents=True)
    m=json.loads((root/'manifest.json').read_text(encoding='utf8'))
    rows=[json.loads(x) for x in (root/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    histories=defaultdict(list)
    for row in rows:
        for name,data in row['cameras'].items():
            if name=='back':continue
            for obj in data.get('objects',[]):
                x,y,w,h=obj['bbox']
                if obj.get('track_status')=='confirmed' and obj['kind'] in ('car','truck','bus','motorcycle','bicycle') and w>=30 and h>=(24 if name=='front' else 50):
                    histories[obj['track_id']].append((name,row['time_s'],obj,data.get('frame_index')))
    directory=Path(m['source_front_path']).parent;readers={n:cv2.VideoCapture(str(directory/v['file'])) for n,v in m['cameras'].items()}
    from core.scene_rectification import CameraRectifier
    rectifiers={n:CameraRectifier(v['rectification']['profile'],v['width'],v['height'])
                for n,v in m['cameras'].items() if v.get('rectification',{}).get('status')=='applied'}
    pages=[];index=[]
    try:
        for source,history in histories.items():
            if len(history)<8:continue
            tiles=[]
            for name,t,obj,frame_index in [history[0],history[len(history)//2],history[-1]]:
                if frame_index is not None:readers[name].set(cv2.CAP_PROP_POS_FRAMES,frame_index)
                else:readers[name].set(cv2.CAP_PROP_POS_MSEC,t*1000)
                ok,f=readers[name].read()
                if not ok:raise ValueError(f'Missing source {name} {t}')
                if name in rectifiers:f=rectifiers[name].apply(f)
                x,y,w,h=obj['bbox']
                x1=max(0,int(x-.3*w));y1=max(0,int(y-.3*h));x2=min(f.shape[1],int(x+1.3*w));y2=min(f.shape[0],int(y+1.3*h))
                cv2.rectangle(f,(round(x),round(y)),(round(x+w),round(y+h)),(0,255,0),2)
                crop=f[y1:y2,x1:x2];tile=np.zeros((130,300,3),np.uint8)
                scale=min(300/crop.shape[1],108/crop.shape[0]);r=cv2.resize(crop,(max(1,round(crop.shape[1]*scale)),max(1,round(crop.shape[0]*scale))))
                tile[22:22+r.shape[0],:r.shape[1]]=r;cv2.putText(tile,f'{source} {t:.1f}s',(4,16),0,.4,(0,255,255),1);tiles.append(tile)
            pages.append(np.hstack(tiles));index.append(dict(source=source,frames=len(history),kind=history[0][2]['kind']))
        for j in range(0,len(pages),10):cv2.imencode('.jpg',np.vstack(pages[j:j+10]))[1].tofile(str(output/f'identities-{j//10+1}.jpg'))
        (output/'identity-index.json').write_text(json.dumps(index,indent=2),encoding='utf8')
    finally:
        for r in readers.values():r.release()
    return len(index)

if __name__=='__main__':print(review(sys.argv[1],sys.argv[2]))
