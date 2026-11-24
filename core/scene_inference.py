"""Optional DirectML inference; never loads CUDA and always records fallback."""
from pathlib import Path
import sys
import cv2
import numpy as np


class InferenceEngine:
    def __init__(self, model, size, backend='cpu', device_id=0):
        if backend not in ('cpu','directml','auto'):raise ValueError('Unknown inference backend')
        if device_id<0:raise ValueError('GPU adapter index must be nonnegative')
        self.model,self.size=model,size
        self.session=None;self.net=None
        self.info={'requested':backend,'effective':'opencv_cpu','device_id':device_id,'fallback_reason':None}
        if backend!='cpu':
            try:
                # Optional isolated installation, without replacing the editor's packages.
                local=Path(__file__).resolve().parents[2]/'scene-runtime-directml'
                if local.is_dir() and 'onnxruntime' not in sys.modules:sys.path.insert(0,str(local))
                import onnxruntime as ort
                if 'DmlExecutionProvider' not in ort.get_available_providers():
                    raise RuntimeError('DirectML execution provider is not installed')
                options=ort.SessionOptions();options.enable_mem_pattern=False
                options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
                self.session=ort.InferenceSession(model,sess_options=options,
                    providers=[('DmlExecutionProvider',{'device_id':device_id}),'CPUExecutionProvider'])
                self.session.disable_fallback()
                if 'DmlExecutionProvider' not in self.session.get_providers():
                    raise RuntimeError('DirectML initialization fell back to CPU')
                self.input=self.session.get_inputs()[0].name
                self.session.run(None,{self.input:np.zeros((1,3,size,size),np.float32)})
                self.info.update(effective='directml',runtime_version=ort.__version__,
                    providers=self.session.get_providers())
            except Exception as exc:
                self.session=None;self.info['fallback_reason']=str(exc)
        if self.session is None:self._cpu()

    def _cpu(self):
        self.net=cv2.dnn.readNetFromONNX(self.model)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.info['effective']='opencv_cpu'

    def run(self,blob):
        if self.session is not None:
            try:return self.session.run(None,{self.input:blob})[0]
            except Exception as exc:
                self.info['fallback_reason']='DirectML inference failed: '+str(exc)
                self.session=None;self._cpu()
        self.net.setInput(blob)
        return self.net.forward()

    def run_all(self,blob):
        if self.session is not None:
            try:return self.session.run(None,{self.input:blob})
            except Exception as exc:
                self.info['fallback_reason']='DirectML inference failed: '+str(exc)
                self.session=None;self._cpu()
        self.net.setInput(blob)
        return list(self.net.forward(self.net.getUnconnectedOutLayersNames()))
