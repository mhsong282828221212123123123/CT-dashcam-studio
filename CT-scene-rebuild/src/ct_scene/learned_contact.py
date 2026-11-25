"""Opt-in experimental image-space adapter. Never mutates world geometry."""
from pathlib import Path
import hashlib,json
import numpy as np
from PIL import Image
from .contact_learning import metadata,DIRECTIONS


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
