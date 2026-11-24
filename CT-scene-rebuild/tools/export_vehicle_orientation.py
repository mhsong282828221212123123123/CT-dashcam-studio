"""CPU-only experimental orientation export; not part of app dependencies.

Published tensor state: ruhyadi/yolo3d-lightning v0.6. Architecture is reconstructed
with torchvision modules and strict state matching. No upstream execution,
CUDA, detector replacement, learned XYZ, or runtime pipeline adoption.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torchvision.models import mobilenet_v3_small


class OrientationRegressor(nn.Module):
    def __init__(self):
        super().__init__();self.net=nn.Sequential(mobilenet_v3_small(weights=None).features)
        self.orientation=self.head(28224,256,4);self.confidence=self.head(28224,256,2);self.dimension=self.head(28224,512,3)

    @staticmethod
    def head(features,hidden,outputs):
        return nn.Sequential(nn.Linear(features,hidden),nn.ReLU(True),nn.Dropout(),nn.Linear(hidden,hidden),nn.ReLU(True),nn.Dropout(),nn.Linear(hidden,outputs))

    def forward(self,image):
        features=self.net(image).flatten(1)
        orientation=nn.functional.normalize(self.orientation(features).reshape(-1,2,2),dim=2)
        return orientation,self.confidence(features),self.dimension(features)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--weights',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    assert torch.version.cuda is None,'Use the isolated CPU export environment'
    expected='a1e4a12870566967d248c28e9dcc6f40945142d1b0ecc3d11636668becbdae80'
    sha=hashlib.sha256(a.weights.read_bytes()).hexdigest()
    if sha!=expected:raise ValueError('Unreviewed model tensor file')
    state=torch.load(a.weights,map_location='cpu',weights_only=True)
    state={k.removeprefix('net.'):v for k,v in state.items()}
    model=OrientationRegressor().cpu().eval();model.load_state_dict(state,strict=True)
    assert all(p.device.type=='cpu' for p in model.parameters())
    image=np.random.default_rng(42).normal(0,1,(1,3,224,224)).astype('float32');tensor=torch.from_numpy(image)
    with torch.no_grad():reference=[x.numpy() for x in model(tensor)]
    if not all(np.isfinite(x).all() for x in reference):raise ValueError('Nonfinite CPU model')
    torch.onnx.export(model,tensor,str(a.output),opset_version=13,input_names=['image'],output_names=['orientation','confidence','dimension'],dynamo=False)
    np.save(a.output.with_suffix('.input.npy'),image)
    np.savez(a.output.with_suffix('.reference.npz'),**dict(zip(['orientation','confidence','dimension'],reference)))
    record=dict(source='https://github.com/ruhyadi/yolo3d-lightning/releases/tag/v0.6',weights_sha256=sha,onnx_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),
                execution_device='cpu',torch=torch.__version__,cuda_build=torch.version.cuda,runtime_adopted=False,
                input='RGB tight crop, cubic resize224x224, ImageNet mean/std',output_shapes=[list(x.shape) for x in reference],
                dimensions_and_xyz_used=False,scope='experimental orientation candidate; requires ONNX parity and source review')
    a.output.with_suffix('.export.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))


if __name__=='__main__':main()
