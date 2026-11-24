"""Exercise only the editor's process adapter, without running legacy analysis."""
import ast,os,subprocess
from pathlib import Path
from types import SimpleNamespace
root=Path(__file__).resolve().parents[1];path=root.parent/'CT-dashcam-studio/ui/main_window.py'
tree=ast.parse(path.read_text(encoding='utf-8-sig'));cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CTDashcamStudio')
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='open_scene_v2')
calls=[]
namespace={'__file__':str(path),'os':os,'subprocess':SimpleNamespace(Popen=lambda *args,**kwargs:calls.append((args,kwargs)),CREATE_NO_WINDOW=getattr(subprocess,'CREATE_NO_WINDOW',0)),
           'QMessageBox':SimpleNamespace(warning=lambda *args:(_ for _ in ()).throw(AssertionError(args)))}
exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),namespace)
source='E:/tesla/example-front.mp4';stub=SimpleNamespace(is_exporting=False,timer=SimpleNamespace(isActive=lambda:False),active_clip_list=[{'cams':{'front':source}}],current_active_clip_idx=0)
namespace['open_scene_v2'](stub)
args,kwargs=calls[0];command=args[0]
assert command[1:3]==['-m','ct_scene.review_app'] and command[-2:]==['--front',source]
assert kwargs['env']['PYTHONPATH']==str(root/'src') and kwargs['cwd']==str(root)
print('Editor adapter: isolated v2 process, selected source forwarded, no legacy engine import')
