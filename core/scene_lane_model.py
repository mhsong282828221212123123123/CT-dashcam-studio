"""Experimental UFLDv2 TuSimple ONNX adapter, not yet enabled in the editor.

Model preprocessing/anchors follow the upstream TuSimple configuration.
Outputs are image hypotheses, not verified paint or physical road boundaries.
"""
from pathlib import Path
import sys
import cv2
import numpy as np


class LaneModel:
    def __init__(self,path,backend='directml'):
        if backend not in ('directml','cpu'):
            raise ValueError('Only DirectML and CPU are supported')
        runtime=Path(__file__).resolve().parents[2]/'scene-runtime-directml'
        if runtime.is_dir() and 'onnxruntime' not in sys.modules:
            sys.path.insert(0,str(runtime))
        import onnxruntime as ort
        options=ort.SessionOptions()
        options.enable_mem_pattern=False
        options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        providers=['CPUExecutionProvider']
        if backend=='directml':
            if 'DmlExecutionProvider' not in ort.get_available_providers():
                raise RuntimeError('DirectML is unavailable')
            providers.insert(0,'DmlExecutionProvider')
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=providers)
        self.session.disable_fallback()
        inputs=self.session.get_inputs()
        if len(inputs)!=1 or inputs[0].shape!=[1,3,320,800]:
            raise ValueError('This adapter requires the 800x320 TuSimple model')
        shapes={o.name:o.shape for o in self.session.get_outputs()}
        if shapes!={'loc_row':[1,100,56,4],'loc_col':[1,100,41,4],
                    'exist_row':[1,2,56,4],'exist_col':[1,2,41,4]}:
            raise ValueError('Unexpected output contract')
        self.input=inputs[0].name
        self.names=[o.name for o in self.session.get_outputs()]
        self.providers=self.session.get_providers()

    def detect(self,frame,viewport='full'):
        height,width=frame.shape[:2]
        if viewport not in ('full','bottom_16_9'):
            raise ValueError('Unknown viewport')
        top=max(0,height-round(width*9/16)) if viewport=='bottom_16_9' else 0
        frame=frame[top:]
        height=frame.shape[0]
        # Resize whole image to 400 high then take lower 320, as upstream
        # evaluation does. Coordinates below remain relative to original image.
        rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        resized=cv2.resize(rgb,(800,400),interpolation=cv2.INTER_LINEAR)[-320:]
        normalized=(resized.astype(np.float32)/255.-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        blob=np.ascontiguousarray(normalized.transpose(2,0,1)[None])
        predictions=dict(zip(self.names,self.session.run(None,{self.input:blob})))
        lanes=decode_lanes(predictions,width,height)
        for lane in lanes:
            for point in lane['points']:point[1]+=top
        return lanes


def decode_lanes(predictions,width,height):
    lanes=[]
    for axis,indices,anchors in [('row',(1,2),np.linspace(160/720,710/720,56)),
                                  ('col',(0,3),np.linspace(0,1,41))]:
        locations=predictions['loc_'+axis][0]
        existence=predictions['exist_'+axis][0]
        for lane in indices:
            valid=existence[:,:,lane].argmax(0)==1
            if valid.sum()<=len(valid)/(2 if axis=='row' else 4):
                continue
            points=[]
            for anchor in np.flatnonzero(valid):
                logits=locations[:,anchor,lane]
                peak=int(logits.argmax())
                grid=np.arange(max(0,peak-1),min(len(logits),peak+2))
                weights=np.exp(logits[grid]-logits[grid].max())
                coordinate=(float(weights@grid/weights.sum())+.5)/(len(logits)-1)
                point=(coordinate*width,anchors[anchor]*height) if axis=='row' else (anchors[anchor]*width,coordinate*height)
                if 0<=point[0]<width and 0<=point[1]<height:
                    points.append([float(v) for v in point])
            if len(points)>=4:
                points.sort(key=lambda p:p[1])
                lanes.append(dict(id=f'ufld-{lane}',points=points,source='ufldv2_tusimple',
                                  boundary_type='unknown',extrapolated=False,
                                  evidence='model_hypothesis_not_verified_paint'))
    return lanes
