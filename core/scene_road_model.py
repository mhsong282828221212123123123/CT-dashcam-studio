"""Optional local road evidence model. No CUDA or network requests."""
from pathlib import Path
import hashlib
import sys
import cv2
import numpy as np
from core.scene_road_evidence import segmented_curves,road_cross_sections


class RoadEvidenceDetector:
    def __init__(self,path,backend='directml'):
        if backend not in ('directml','cpu'):raise ValueError('Unsupported backend')
        runtime=Path(__file__).resolve().parents[2]/'scene-runtime-directml'
        if runtime.is_dir() and 'onnxruntime' not in sys.modules:sys.path.insert(0,str(runtime))
        import onnxruntime as ort
        options=ort.SessionOptions();options.enable_mem_pattern=False
        options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        providers=['CPUExecutionProvider']
        if backend=='directml':
            if 'DmlExecutionProvider' not in ort.get_available_providers():raise RuntimeError('DirectML unavailable')
            providers.insert(0,'DmlExecutionProvider')
        self.session=ort.InferenceSession(str(path),sess_options=options,providers=providers)
        self.session.disable_fallback()
        inputs=self.session.get_inputs()
        if len(inputs)!=1 or inputs[0].shape!=[1,3,640,640]:raise ValueError('Expected 640x640 YOLOP model')
        shapes={o.name:o.shape for o in self.session.get_outputs()}
        self.v2=shapes.get('road')==[1,2,640,640] and shapes.get('lane')==[1,1,640,640]
        if not self.v2:
            for key in ('drive_area_seg','lane_line_seg'):
                if shapes.get(key)!=[1,2,640,640]:raise ValueError('Unexpected segmentation outputs')
        self.input=inputs[0].name
        self.info=dict(model='YOLOPv2' if self.v2 else 'YOLOP',evidence_version=2,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                       providers=self.session.get_providers(),status='experimental')

    def detect(self,frame,objects=()):
        h,w=frame.shape[:2];scale=min(640/w,640/h);rw,rh=round(w*scale),round(h*scale)
        px,py=(640-rw)//2,(640-rh)//2
        canvas=np.full((640,640,3),114,np.uint8)
        canvas[py:py+rh,px:px+rw]=cv2.resize(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB),(rw,rh))
        blob=canvas.astype(np.float32)/255.
        if not self.v2:blob=(blob-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        masks=[]
        outputs=self.session.run(['road','lane'] if self.v2 else ['drive_area_seg','lane_line_seg'],
                                  {self.input:np.ascontiguousarray(blob.transpose(2,0,1)[None])})
        for output in outputs:
            scores=np.stack([cv2.resize(channel[py:py+rh,px:px+rw],(w,h)) for channel in output[0]])
            masks.append((scores[0]>.5).astype(np.uint8) if len(scores)==1 else scores.argmax(0).astype(np.uint8))
        road,paint=masks
        self.last_masks=(road,paint)
        sections=road_cross_sections(road)
        edges=[]
        for side in (0,1):
            groups=[];group=[]
            for section in sections:
                x,y=section[side]
                blocked=any(o['bbox'][0]-8<x<o['bbox'][0]+o['bbox'][2]+8 and
                            o['bbox'][1]-8<y<o['bbox'][1]+o['bbox'][3]+8 for o in objects)
                if x<=3 or x>=w-4 or blocked:
                    if len(group)>=3:groups.append(group)
                    group=[]
                else:group.append([x,y])
            if len(group)>=3:groups.append(group)
            edges.extend(groups)
        return dict(source='camera_road_segmentation',
                    lanes=segmented_curves(paint,road,objects),
                    cross_sections=sections,edge_segments=edges,
                    boundary_semantics='road_extent_estimate_not_verified_edge',
                    image_size=[w,h])
