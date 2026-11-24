"""CPU-only export of official YOLOPv2 TorchScript road heads."""
import os
os.environ['CUDA_VISIBLE_DEVICES']='-1'
import torch


class RoadHeads(torch.nn.Module):
    def __init__(self,net):super().__init__();self.net=net
    def forward(self,x):
        result=self.net(x)
        return result[1],result[2]


if __name__=='__main__':
    torch.set_num_threads(4)
    net=torch.jit.load('../scene-models/model-review/yolopv2.pt',map_location='cpu').eval()
    wrapper=torch.jit.script(RoadHeads(net).eval())
    torch.onnx.export(wrapper,torch.zeros(1,3,640,640),'../scene-models/model-review/yolopv2-roads.onnx',
        input_names=['images'],output_names=['road','lane'],opset_version=17,do_constant_folding=True)
    print('YOLOPv2 road heads exported on CPU')
