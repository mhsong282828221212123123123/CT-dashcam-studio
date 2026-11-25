"""Reuse the existing local vehicle backbone on CPU; no download or CUDA."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from torch import nn
from torchvision.models import mobilenet_v3_small
p=argparse.ArgumentParser();p.add_argument('--annotations',type=Path,required=True);p.add_argument('--weights',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
torch.set_num_threads(2);torch.manual_seed(20261125)
state=torch.load(a.weights,map_location='cpu',weights_only=True)
features=mobilenet_v3_small(weights=None).features
prefix='net.net.0.';features.load_state_dict({k[len(prefix):]:v for k,v in state.items() if k.startswith(prefix)},strict=True)
encoder=nn.Sequential(features,nn.AvgPool2d(4,stride=3),nn.Flatten()).eval()
records=json.loads(a.annotations.read_text())['records'];vectors=[]
with torch.inference_mode():
 for i,r in enumerate(records):
  image=np.array(Image.open(r['crop']).convert('RGB').resize((224,224),Image.Resampling.BICUBIC)).astype('float32')/255
  x=torch.from_numpy(((image-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)).transpose(2,0,1)[None])
  vectors.append(encoder(x).numpy()[0])
  if i%20==0:print('CPU features',i,'/',len(records),flush=True)
np.savez_compressed(a.output/'features.npz',ids=np.array([r['id'] for r in records]),features=np.array(vectors))
torch.onnx.export(encoder,torch.zeros(1,3,224,224),a.output/'contact-encoder.onnx',input_names=['rgb_normalized'],output_names=['features'],dynamic_axes={'rgb_normalized':{0:'batch'},'features':{0:'batch'}},opset_version=17)
(a.output/'provenance.json').write_text(json.dumps(dict(source_weights=str(a.weights),source_weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),annotations_sha256=hashlib.sha256(a.annotations.read_bytes()).hexdigest(),torch=torch.__version__,device='cpu',cuda_used=False,backbone='existing KITTI-trained mobilenet_v3_small features; frozen',learned_head='separate calibration and image direction heads',target='image contacts only, not metric XYZ'),indent=2))
