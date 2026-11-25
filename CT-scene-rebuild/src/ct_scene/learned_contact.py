"""Opt-in experimental image-space adapter. Never mutates world geometry."""
from pathlib import Path
import hashlib,json
import numpy as np
from PIL import Image
from .contact_learning import metadata,rich_metadata,flip_record,flip_contact_uv,DIRECTIONS


def preprocess(crop):
    image=np.array(Image.fromarray(crop).convert('RGB').resize((224,224),Image.Resampling.BICUBIC)).astype('float32')/255
    x=(image-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
    return np.ascontiguousarray(x.transpose(2,0,1)[None])


class LearnedContact:
    def __init__(self,directory):
        import onnxruntime as ort
        directory=Path(directory);self.report=json.loads((directory/'manifest.json').read_text())
        for name,digest in self.report['files_sha256'].items():
            if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:raise ValueError('Model package checksum mismatch')
        self.weights=np.load(directory/'adapter.npz',allow_pickle=False)
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        self.encoder=ort.InferenceSession(str(directory/'contact-encoder.onnx'),options,providers=['CPUExecutionProvider'])

    def predict(self,crop,record):
        if self.report.get('schema')=='ct_scene.experimental_contact_adapter_v5':
            cfg=self.report['adapter_config']
            batch=np.concatenate([preprocess(crop),preprocess(crop[:,::-1,:])],axis=0)
            out=self.encoder.run(None,{'rgb_normalized':batch})[0].astype(float)
            u_crop=0.5*(out[0,0]+(1.0-out[1,0])) if cfg.get('flip_tta',True) else out[0,0]
            v_box=0.5*(out[0,1]+out[1,1]) if cfg.get('flip_tta',True) else out[0,1]
            cr=np.asarray(record['crop_rect_xyxy'],float);b=np.asarray(record['bbox_xyxy'],float)
            u_box=((cr[0]+u_crop*(cr[2]-cr[0]))-b[0])/(b[2]-b[0])
            return dict(contact_uv=[float(u_box),float(v_box)],direction='disabled',
                applied=False,status='experimental_not_approved',metric_position_available=False,
                reason=self.report['adoption_gate'],confidence_calibrated=False)
        if self.report.get('schema')=='ct_scene.experimental_contact_adapter_v4':
            cfg=self.report['adapter_config']
            batch=np.concatenate([preprocess(crop),preprocess(crop[:,::-1,:])],axis=0)
            uv_c,feats=self.encoder.run(None,{'rgb_normalized':batch})
            uv_c=uv_c.astype(float);feats=feats.astype(float)
            uv_crop=0.5*(uv_c[0]+flip_contact_uv(uv_c[1])) if cfg.get('flip_tta',True) else uv_c[0]
            cr=np.asarray(record['crop_rect_xyxy'],float);b=np.asarray(record['bbox_xyxy'],float)
            xy=cr[:2]+uv_crop*(cr[2:]-cr[:2])
            contact=(xy-b[:2])/(b[2:]-b[:2])
            z=np.clip((feats-self.weights['feature_mean'])/self.weights['feature_scale'],-6,6)/np.sqrt(feats.shape[1])
            md=np.vstack([metadata(record),metadata(flip_record(record))])
            sd=self.report['selected']['direction']['feature_strength']
            pd=np.c_[z*sd,md]@self.weights['direction_w']+self.weights['direction_b']
            d_logits=0.5*(pd[0]+pd[1][[0,1,3,2]]) if cfg.get('flip_tta',True) else pd[0]
            return dict(contact_uv=contact.tolist(),direction=DIRECTIONS[int(np.argmax(d_logits))],
                applied=False,status='experimental_not_approved',metric_position_available=False,
                reason=self.report['adoption_gate'],confidence_calibrated=False)
        if self.report.get('schema')=='ct_scene.experimental_contact_adapter_v3':
            cfg=self.report['adapter_config']
            batch=np.concatenate([preprocess(crop),preprocess(crop[:,::-1,:])],axis=0)
            feats=self.encoder.run(None,{'rgb_normalized':batch})[0].astype(float)
            z=np.clip((feats-self.weights['feature_mean'])/self.weights['feature_scale'],-6,6)
            rec_f=flip_record(record)
            c0,c1=cfg['contact_slice'];d0,d1=cfg['direction_slice']
            zc=z[:,c0:c1]/np.sqrt(c1-c0);zd=z[:,d0:d1]/np.sqrt(d1-d0)
            mc_fn=rich_metadata if cfg.get('rich_meta_contact',True) else metadata
            mc=np.vstack([mc_fn(record),mc_fn(rec_f)])
            md=np.vstack([metadata(record),metadata(rec_f)])
            sc=self.report['selected']['contact']['feature_strength']
            sd=self.report['selected']['direction']['feature_strength']
            pc=np.c_[zc*sc,mc]@self.weights['contact_w']+self.weights['contact_b']
            pd=np.c_[zd*sd,md]@self.weights['direction_w']+self.weights['direction_b']
            if cfg.get('height_space_contact',True):
                b=np.asarray(record['bbox_xyxy'],float);ratio=(b[2]-b[0])/(b[3]-b[1])
                pc=np.c_[pc[:,0]/ratio,pc[:,1]]
            c_o=pc[0]+[.5,1.0];c_f=flip_contact_uv(pc[1]+[.5,1.0])
            contact=0.5*(c_o+c_f) if cfg.get('flip_tta',True) else c_o
            d_logits=0.5*(pd[0]+pd[1][[0,1,3,2]]) if cfg.get('flip_tta',True) else pd[0]
            return dict(contact_uv=contact.tolist(),direction=DIRECTIONS[int(np.argmax(d_logits))],
                applied=False,status='experimental_not_approved',metric_position_available=False,
                reason=self.report['adoption_gate'],confidence_calibrated=False)
        f=self.encoder.run(None,{'rgb_normalized':preprocess(crop)})[0][0].astype(float)
        z=np.clip((f-self.weights['feature_mean'])/self.weights['feature_scale'],-6,6)/np.sqrt(len(f))
        predictions={}
        for name in ['contact','direction']:
            strength=self.report['selected'][name]['feature_strength'];x=np.r_[z*strength,metadata(record)]
            predictions[name]=x@self.weights[name+'_w']+self.weights[name+'_b']
        contact=predictions['contact']+[.5,1]
        return dict(contact_uv=contact.tolist(),direction=DIRECTIONS[int(np.argmax(predictions['direction']))],
            applied=False,status='experimental_not_approved',metric_position_available=False,
            reason=self.report['adoption_gate'],confidence_calibrated=False)
