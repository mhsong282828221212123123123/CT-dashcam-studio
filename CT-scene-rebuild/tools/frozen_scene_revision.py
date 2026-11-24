"""Read the explicit composition dependency list without running a renderer."""
import ast
from pathlib import Path
from ct_scene.artifacts import digest


def scene_revision(engine):
    root=Path(engine)/'src/ct_scene'
    tree=ast.parse((root/'surrounding.py').read_text(encoding='utf-8'))
    run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    names=None
    for node in run.body:
        if isinstance(node,ast.Assign):
            targets=[n.id for n in node.targets if isinstance(n,ast.Name)]
            if 'revision_names' in targets:names=ast.literal_eval(node.value)
            if 'started_revision' in targets:break
        elif isinstance(node,ast.Expr) and isinstance(node.value,ast.Call):
            f=node.value.func
            if isinstance(f,ast.Attribute) and isinstance(f.value,ast.Name) and f.value.id=='revision_names' and f.attr=='append':
                names.append(ast.literal_eval(node.value.args[0]))
    if not names or not all(isinstance(n,str) for n in names):raise ValueError('Explicit revision dependency list required')
    return digest(b''.join((root/name).read_bytes() for name in names))
