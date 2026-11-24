"""Standalone context/GLB geometry spike, NOT a calibrated reconstruction renderer."""
from pathlib import Path
import json
import hashlib
import numpy as np
import moderngl
import trimesh
from PIL import Image

root = Path(__file__).resolve().parents[1]
asset = root.parent/'CT-dashcam-studio/assets/scene/models/tesla_fsd_sedan.glb'
if not asset.exists():
    asset = next((root.parent/'CT-dashcam-studio/assets/scene').rglob('tesla_fsd_sedan.glb'))
out = root/'runs/renderer-spike-001'
out.mkdir(parents=True, exist_ok=False)
try:
    ctx = moderngl.create_context(standalone=True, require=330)
    scene = trimesh.load_scene(asset)
    arrays = []
    for node in scene.graph.nodes_geometry:
        T, key = scene.graph[node]
        mesh = scene.geometry[key]
        p = trimesh.transform_points(mesh.vertices, T)
        n = mesh.vertex_normals @ np.linalg.inv(T[:3,:3])
        n /= np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-8)
        arrays.append(np.column_stack([p,n])[mesh.faces.ravel()])
    vertices = np.concatenate(arrays)
    lo, hi = vertices[:,:3].min(0),vertices[:,:3].max(0)
    vertices[:,:3] = (vertices[:,:3]-(lo+hi)/2)/(hi-lo).max()*1.4
    a,b = np.radians(40),np.radians(35)
    Ry = np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    Rx = np.array([[1,0,0],[0,np.cos(b),-np.sin(b)],[0,np.sin(b),np.cos(b)]])
    R = Rx@Ry
    vertices[:,:3] = vertices[:,:3]@R.T
    vertices[:,3:] = vertices[:,3:]@R.T
    program = ctx.program(vertex_shader='''#version 330
in vec3 position; in vec3 normal; out float light;
void main(){ gl_Position=vec4(position.xy,-position.z,1.0);light=0.35+0.65*max(dot(normalize(normal),normalize(vec3(1,2,3))),0.0); }
''', fragment_shader='''#version 330
in float light; uniform float alpha; out vec4 color;
void main(){color=vec4(vec3(0.45,0.48,0.5)*light,alpha);}
''')
    buffer = ctx.buffer(vertices.astype('f4').tobytes())
    vao = ctx.vertex_array(program, [(buffer,'3f 3f','position','normal')])
    fbo = ctx.simple_framebuffer((768,768), components=4)
    fbo.use()
    ctx.enable(moderngl.DEPTH_TEST | moderngl.BLEND)
    ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA
    images=[]
    for opacity in [1.0,0.35]:
        fbo.clear(.95,.95,.95,1)
        program['alpha'].value=opacity
        vao.render()
        pixels=np.frombuffer(fbo.read(components=3,alignment=1),np.uint8).reshape(768,768,3)[::-1].copy()
        assert np.sum(np.min(pixels,axis=2)<200)>1000,'Blank model rendering'
        images.append(Image.fromarray(pixels))
    proof=Image.new('RGB',(1536,768))
    proof.paste(images[0],(0,0));proof.paste(images[1],(768,0));proof.save(out/'proof.png')
    result=dict(status='CONTEXT_AND_GEOMETRY_PASS',gl_vendor=ctx.info['GL_VENDOR'],gl_renderer=ctx.info['GL_RENDERER'],
                asset=str(asset),asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),triangles=len(vertices)//3,
                limitations=['Basic geometry/alpha only; full GLB materials, lamp nodes and calibrated world rendering NOT_EVALUATED'])
except Exception as exc:
    result=dict(status='FAIL',error=repr(exc))
(out/'report.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
