"""Reuse the existing local vehicle backbone on CPU; no download or CUDA."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from torch import nn
from torchvision.models import mobilenet_v3_small


class MultiPoolEncoder(nn.Module):
    """Frozen MobileNetV3-small features with whole-car 2x2 + bottom-half horizontal strips."""
    def __init__(self, features):
        super().__init__()
        self.features = features
        self.avg2x2 = nn.AvgPool2d(4, stride=3)

    def forward(self, x):
        m = self.features(x)
        whole = self.avg2x2(m).flatten(1)
        glob = m.mean(dim=(2, 3))
        bot = m[:, :, 3:, :]
        b0 = bot[:, :, :, 0:2].mean(dim=(2, 3))
        b1 = bot[:, :, :, 2:4].mean(dim=(2, 3))
        b2 = bot[:, :, :, 3:5].mean(dim=(2, 3))
        b3 = bot[:, :, :, 5:7].mean(dim=(2, 3))
        lr = bot[:, :, :, 4:].mean(dim=(2, 3)) - bot[:, :, :, :3].mean(dim=(2, 3))
        return torch.cat([whole, glob, b0, b1, b2, b3, lr], dim=1)


def to_tensor(im):
    image = np.array(im.resize((224, 224), Image.Resampling.BICUBIC)).astype('float32') / 255
    return torch.from_numpy(((image - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)).transpose(2, 0, 1)[None])


p = argparse.ArgumentParser()
p.add_argument('--annotations', type=Path, required=True)
p.add_argument('--weights', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--encoder-version', choices=['v1', 'v3'], default='v1')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
torch.set_num_threads(2); torch.manual_seed(20261125)
state = torch.load(a.weights, map_location='cpu', weights_only=True)
features = mobilenet_v3_small(weights=None).features
prefix = 'net.net.0.'; features.load_state_dict({k[len(prefix):]: v for k, v in state.items() if k.startswith(prefix)}, strict=True)
encoder = (MultiPoolEncoder(features) if a.encoder_version == 'v3' else nn.Sequential(features, nn.AvgPool2d(4, stride=3), nn.Flatten())).eval()
records = json.loads(a.annotations.read_text())['records']; vectors = []; vectors_flip = []
with torch.inference_mode():
    for i, r in enumerate(records):
        im = Image.open(r['crop']).convert('RGB')
        vectors.append(encoder(to_tensor(im)).numpy()[0])
        if a.encoder_version == 'v3':
            vectors_flip.append(encoder(to_tensor(im.transpose(Image.Transpose.FLIP_LEFT_RIGHT))).numpy()[0])
        if i % 20 == 0: print('CPU features', i, '/', len(records), flush=True)
payload = dict(ids=np.array([r['id'] for r in records]), features=np.array(vectors))
if a.encoder_version == 'v3': payload['features_flip'] = np.array(vectors_flip)
np.savez_compressed(a.output / 'features.npz', **payload)
torch.onnx.export(encoder, torch.zeros(1, 3, 224, 224), a.output / 'contact-encoder.onnx', input_names=['rgb_normalized'], output_names=['features'], dynamic_axes={'rgb_normalized': {0: 'batch'}, 'features': {0: 'batch'}}, opset_version=17)
(a.output / 'provenance.json').write_text(json.dumps(dict(source_weights=str(a.weights), source_weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(), annotations_sha256=hashlib.sha256(a.annotations.read_bytes()).hexdigest(), encoder_version=a.encoder_version, torch=torch.__version__, device='cpu', cuda_used=False, backbone='existing KITTI-trained mobilenet_v3_small features; frozen', learned_head='separate calibration and image direction heads', target='image contacts only, not metric XYZ'), indent=2))
