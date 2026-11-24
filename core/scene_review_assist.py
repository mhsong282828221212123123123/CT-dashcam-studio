"""Explicit visual-review corrections, propagated only with checked image motion.

This is not detector accuracy: every assisted object retains its review provenance.
No reference boxes are applied to another clip or beyond their reviewed time window.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from core.scene_analysis import iou


class ReviewedObservationAssist:
    def __init__(self,path,front_file):
        self.path=Path(path)
        self.reference=json.loads(self.path.read_text(encoding='utf-8'))
        if self.reference['front_file']!=front_file: raise ValueError('Review correction clip mismatch')
        self.frames=self.reference['frames']; self.active={}; self.gray={}
        self.end=max(f['time_s'] for f in self.frames)

    def update(self,camera,frame,t,objects):
        if t>self.end+.05: return objects,[],[]
        expected=self.reference.get('image_size',[1280,960])
        if [frame.shape[1],frame.shape[0]]!=expected: raise ValueError('Review reference image dimensions do not match')
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        states=self.active.setdefault(camera,{})
        previous=self.gray.get(camera)
        for key,state in list(states.items()):
            max_age=.6 if state['target']['kind']=='person' else 4.
            if previous is None or t-state['anchor_time']>max_age or t-state['time']>.5:
                del states[key]; continue
            x,y,w,h=state['bbox']; mask=np.zeros_like(gray)
            cv2.rectangle(mask,(max(0,int(x+3)),max(0,int(y+3))),(min(gray.shape[1]-1,int(x+w-3)),min(gray.shape[0]-1,int(y+h-3))),255,-1)
            points=cv2.goodFeaturesToTrack(previous,60,.02,7,mask=mask)
            if points is None or len(points)<6: del states[key]; continue
            moved,ok,_=cv2.calcOpticalFlowPyrLK(previous,gray,points,None)
            back,ok2,_=cv2.calcOpticalFlowPyrLK(gray,previous,moved,None)
            valid=(ok.ravel()!=0)&(ok2.ravel()!=0)&(np.linalg.norm(points-back,axis=2).ravel()<1.5)
            if valid.sum()<6: del states[key]; continue
            shifts=(moved-points).reshape(-1,2)[valid]; shift=np.median(shifts,axis=0)
            coherent=np.linalg.norm(shifts-shift,axis=1)<3
            if coherent.mean()<.65 or np.linalg.norm(shift)>40: del states[key]; continue
            state['bbox']=[float(x+shift[0]),float(y+shift[1]),w,h];state['time']=t
        anchors=[f for f in self.frames if f['camera']==camera and abs(f['time_s']-t)<.025]
        for anchor in anchors:
            for target in anchor['targets']:
                if target['status']=='confirmed':
                    states[target['id']]={'bbox':list(target['bbox']),'time':t,'anchor_time':t,'target':target}
        self.gray[camera]=gray
        result=[dict(o) for o in objects]; evidence=[]
        for key,state in states.items():
            target=state['target']; bbox=state['bbox']
            matches=[(iou(bbox,o['bbox']),i) for i,o in enumerate(result) if o.get('source')!='visual_review_flow']
            score,index=max(matches,default=(0,None))
            subtype=target.get('subtype')
            kind='person' if target['kind']=='person' else ('truck' if subtype=='small_truck' else 'car')
            provenance={'review_target':key,'review_anchor_time_s':state['anchor_time'],
                        'review_method':'anchor' if t==state['anchor_time'] else 'forward_backward_optical_flow'}
            if target.get('association_only'):
                provenance.update(association_only=True,bbox_scope='visible_partial_region')
            if score>=.3:
                obj=result[index]; obj['model_kind']=obj['kind'];obj['kind']=kind
                obj['track_status']='confirmed';obj.update(provenance)
                if subtype=='SUV':obj['render_kind']='suv'
                if subtype=='small_truck':obj['render_kind']='small_truck'
            else:
                result.append({'kind':kind,'confidence':1.,'confidence_type':'review_status_not_model_probability',
                    'bbox':bbox,'track_id':f'review:{camera}:{key}','track_status':'confirmed',
                    'observation_count':1,'source':'visual_review_flow',**provenance,
                    'render_kind':'suv' if subtype=='SUV' else ('small_truck' if subtype=='small_truck' else kind)})
                evidence.append({'id':key,'action':'added_reviewed_observation','method':provenance['review_method']})
        nearest=min((f for f in self.frames if f['camera']==camera),key=lambda f:abs(f['time_s']-t),default=None)
        if nearest:
            kept=[]
            for obj in result:
                bx,by,bw,bh=obj['bbox']; ignored=False
                for region in nearest.get('ignored_regions',[]):
                    x,y,w,h=region['bbox']
                    overlap=max(0,min(bx+bw,x+w)-max(bx,x))*max(0,min(by+bh,y+h)-max(by,y))
                    if overlap/max(1,bw*bh)>.35 and 'review_target' not in obj:
                        evidence.append({'id':obj['track_id'],'action':'excluded_reviewed_host_region'});ignored=True;break
                if not ignored:kept.append(obj)
            result=kept
        signals=[dict(s,source='reviewed_candidate_region',state='unknown',reason='presence_or_color_not_confirmed')
                 for s in (nearest.get('signals',[]) if nearest else [])]
        return result,signals,evidence
