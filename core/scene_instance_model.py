"""YOLO26 exported non-end-to-end instance masks in original image pixels."""
from dataclasses import dataclass
import cv2,numpy as np
from core.scene_analysis import SceneDetector,Detection,letterbox


@dataclass
class InstanceDetection(Detection):
    mask_polygon:list=None


class InstanceSceneDetector(SceneDetector):
    def __init__(self,*args,**kwargs):
        if kwargs.get('detail','standard')!='standard':raise ValueError('Mask model currently requires whole-frame mode')
        super().__init__(*args,**kwargs)

    @property
    def inference_info(self):
        return dict(super().inference_info,task='instance_segmentation',mask_coordinates='original_image_pixels')

    def _detect_frame(self,frame):
        padded,transform=letterbox(frame,self.input_size)
        outputs=self.engine.run_all(cv2.dnn.blobFromImage(padded,1/255.,(self.input_size,self.input_size),swapRB=True))
        raw=next((v for v in outputs if v.ndim==3 and v.shape[1]==116),None)
        proto=next((v for v in outputs if v.ndim==4 and v.shape[1]==32),None)
        if raw is None or proto is None:raise ValueError('Expected raw 80-class/32-mask segmentation ONNX export')
        return self.decode_masks(raw,proto,frame.shape[:2],transform)

    def decode_masks(self,raw,proto,shape,transform):
        boxes=self.decode(raw[:,:84,:],shape,transform)
        sx,sy,left,top=transform;h,w=shape
        pw,ph=proto.shape[3],proto.shape[2]
        valid=np.max(raw[0,4:84],axis=0)>=self.threshold
        predictions=raw[0][:,valid].T
        found=[]
        for detection in boxes:
            x,y,bw,bh=detection.bbox
            # Match decoded/NMS box back to its raw mask coefficients. Use clipped
            # boxes, since border detections can have centres outside the image.
            xy=predictions[:,:4]
            lower=np.maximum((xy[:,:2]-xy[:,2:4]/2-[left,top])/[sx,sy],0)
            upper=np.minimum((xy[:,:2]+xy[:,2:4]/2-[left,top])/[sx,sy],[w,h])
            error=np.sum(abs(np.column_stack([lower,upper-lower])-[x,y,bw,bh]),axis=1)
            class_ids=np.argmax(predictions[:,4:84],axis=1)
            error+=np.array([0. if self.classes.get(int(cid))==detection.kind else 1e6 for cid in class_ids])
            error+=abs(np.max(predictions[:,4:84],axis=1)-detection.confidence)*1000
            index=int(np.argmin(error));coeff=predictions[index,84:116]
            logits=(coeff@proto[0].reshape(32,-1)).reshape(ph,pw)
            logits=cv2.resize(logits,(self.input_size,self.input_size))
            rw,rh=round(w*sx),round(h*sy)
            mask=(cv2.resize(logits[top:top+rh,left:left+rw],(w,h))>0).astype(np.uint8)
            crop=np.zeros_like(mask);x0,y0=max(0,int(x)),max(0,int(y))
            x1,y1=min(w,int(np.ceil(x+bw))),min(h,int(np.ceil(y+bh)))
            crop[y0:y1,x0:x1]=mask[y0:y1,x0:x1]
            contours,_=cv2.findContours(crop,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            polygon=[]
            if contours:
                contour=max(contours,key=cv2.contourArea)
                polygon=cv2.approxPolyDP(contour,max(1.,cv2.arcLength(contour,True)*.003),True)[:,0,:].tolist()
            found.append(InstanceDetection(detection.kind,detection.confidence,detection.bbox,polygon))
        return found
