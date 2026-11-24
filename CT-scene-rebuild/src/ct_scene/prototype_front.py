"""Single lead-vehicle 3D prototype using explicit estimated geometry priors.

No camera fusion, lane reconstruction, surveyed calibration or map matching is
claimed. Ego uses own SEI motion; actor observations cannot change ego motion.
"""
from pathlib import Path
from fractions import Fraction
import io
import json
import math
import av
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .artifacts import verify,publish,canonical,digest
from .affine_probe import camera_matrix
from .offline_paths import smooth_positions
from .wheel_scale import SpeedTrace
from .ingest import file_hash


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def model_vertices(asset,clay_color=None,node_filter=None,color_override=None):
    import trimesh
    scene=trimesh.load_scene(asset);arrays=[];all_positions=[]
    # GLB +Z is the model front and +Y is up. This is a proper rotation to W.
    basis=np.array([[0,0,1],[1,0,0],[0,1,0.]])
    for node in scene.graph.nodes_geometry:
        T,key=scene.graph[node];mesh=scene.geometry[key]
        p=trimesh.transform_points(mesh.vertices,T)@basis.T
        all_positions.append(p[mesh.faces.ravel()])
        n=(mesh.vertex_normals@np.linalg.inv(T[:3,:3]))@basis.T;n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-9)
        material=getattr(mesh.visual,'material',None);base=getattr(material,'baseColorFactor',None)
        name=getattr(material,'name','') or ''
        if node_filter is not None and not node_filter(node,name):continue
        color=np.array(base[:3],float) if base is not None else np.array([.8,.82,.84])
        if color.max()>1:color/=255
        if clay_color is not None:color=np.asarray(clay_color,float)
        if color_override is not None:color=np.asarray(color_override,float)
        rgb=np.tile(color,(len(p),1));arrays.append(np.column_stack([p,n,rgb])[mesh.faces.ravel()])
    if not arrays:return np.empty((0,9),'f4')
    result=np.concatenate(arrays).astype('f4')
    # Every subset shares the complete asset's origin, including lamps.
    whole=np.concatenate(all_positions).astype('f4');lo,hi=whole.min(0),whole.max(0)
    result[:,:2]-=(lo[:2]+hi[:2])/2;result[:,2]-=lo[2]
    return result


class Renderer:
    def __init__(self,ego_asset,actor_asset,camera_eye=(-12.,-10.,20.),camera_target=(8.,0.,0.),camera_zoom=1.):
        import moderngl
        self.gl=moderngl;self.ctx=moderngl.create_context(standalone=True,require=330)
        self.program=self.ctx.program(vertex_shader='''#version 330
in vec3 position; in vec3 normal; in vec3 rgb; uniform mat4 mvp; uniform mat4 model; uniform float emissive; out vec3 color;
void main(){gl_Position=mvp*model*vec4(position,1);float l=.55+.45*max(dot(normalize(mat3(model)*normal),normalize(vec3(-.4,-.6,1))),0);color=rgb*mix(l,1.0,emissive);}
''',fragment_shader='''#version 330
in vec3 color; uniform float ghost; out vec4 fragment;
void main(){if(ghost>.5 && (int(gl_FragCoord.x)+int(gl_FragCoord.y))%3==0)discard;fragment=vec4(mix(color,vec3(.93),ghost*.55),1);}
''')
        self.buffers=[];arrays=[model_vertices(a) for a in [ego_asset,actor_asset]]
        self.lengths=[float(np.ptp(a[:,0])) for a in arrays];self.models=[self.mesh(a) for a in arrays]
        ground=[]
        for x,y in [(-100,-100),(150,-100),(150,100),(-100,-100),(150,100),(-100,100)]:ground.append([x,y,-.03,0,0,1,.97,.97,.97])
        self.ground=self.mesh(np.array(ground,'f4'));shadow=[]
        for i in range(40):
            a,b=i*2*np.pi/40,(i+1)*2*np.pi/40
            shadow.extend([[0,0,0,0,0,1,.86,.86,.86],[2.8*np.cos(a),1.1*np.sin(a),0,0,0,1,.97,.97,.97],[2.8*np.cos(b),1.1*np.sin(b),0,0,0,1,.97,.97,.97]])
        self.shadow=self.mesh(np.array(shadow,'f4'));self.fbo=self.ctx.simple_framebuffer((1280,720),components=3)
        eye=np.array(camera_eye,float);target=np.array(camera_target,float);f=target-eye;f/=np.linalg.norm(f);s=np.cross(f,[0,0,1.]);s/=np.linalg.norm(s);u=np.cross(s,f)
        V=np.eye(4);V[:3,:3]=[s,u,-f];V[:3,3]=[-s@eye,-u@eye,f@eye]
        if not np.isfinite(camera_zoom) or camera_zoom<=0:raise ValueError('Positive camera zoom required')
        q=camera_zoom/np.tan(np.radians(52)/2);near,far=.2,300
        P=np.array([[q/(1280/720),0,0,0],[0,q,0,0],[0,0,(far+near)/(near-far),2*far*near/(near-far)],[0,0,-1,0]])
        self.program['mvp'].write((P@V).T.astype('f4').tobytes())
    def mesh(self,data):
        buf=self.ctx.buffer(data.astype('f4').tobytes());self.buffers.append(buf)
        return self.ctx.vertex_array(self.program,[(buf,'3f 3f 3f','position','normal','rgb')])
    def draw(self,mesh,position=(0,0,0),yaw=0,ghost=False,emissive=False):
        T=np.eye(4);T[:3,:3]=rotation(yaw);T[:3,3]=position;self.program['model'].write(T.T.astype('f4').tobytes());self.program['ghost'].value=float(ghost);self.program['emissive'].value=float(emissive);mesh.render()
    def frame(self,relative,yaw):
        self.fbo.use();self.ctx.enable(self.gl.DEPTH_TEST);self.fbo.clear(.97,.97,.97,1,depth=1)
        self.draw(self.ground)
        for pos,angle,mesh in [([-1.2,0,0],0,self.models[0]),(relative,yaw,self.models[1])]:
            self.draw(self.shadow,pos,angle);self.draw(mesh,pos,angle)
        return Image.fromarray(np.frombuffer(self.fbo.read(components=3,alignment=1),np.uint8).reshape(720,1280,3)[::-1].copy())


def run(package,tracks,telemetry,lens_config,ego_asset,actor_asset,destination,camera_height_m=1.3):
    package,tracks,telemetry,destination=map(Path,[package,tracks,telemetry,destination])
    if destination.exists():raise FileExistsError(destination)
    if not np.isfinite(camera_height_m) or camera_height_m<=0:raise ValueError('Positive ground height prior required')
    pm,tm,sm=[verify(p) for p in (package,tracks,telemetry)]
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']):raise ValueError('Track/source mismatch')
    frames=json.loads((package/'frames.json').read_text());front=sorted([f for f in frames if f['camera_id']=='front'],key=lambda f:f['camera_time_s'])
    sei=list(map(json.loads,(telemetry/'telemetry.jsonl').read_text().splitlines()))
    if {r['source_sha256'] for r in sei}!={f['source_sha256'] for f in front}:raise ValueError('Telemetry/source mismatch')
    observations=list(map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()));groups={}
    for row in observations:
        if row['camera_id']=='front':groups.setdefault(row['local_track_id'],[]).append(row)
    candidates=[]
    for key,rows in groups.items():
        boxes=np.array([r['bbox_xyxy'] for r in rows]);centres=(boxes[:,0]+boxes[:,2])/2
        if len(rows)<len(front)*.8 or abs(np.median(centres)-front[0]['width']/2)>front[0]['width']*.15:continue
        score=len(rows)*float(np.median((boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:,1])));candidates.append((score,key,rows))
    if not candidates:raise ValueError('No persistent central lead vehicle')
    _,key,rows=max(candidates,key=lambda c:c[0]);by_frame={r['frame_id']:r for r in rows};selected=[f for f in front if f['frame_id'] in by_frame]
    profile=json.loads(Path(lens_config).read_text())['cameras']['front'];K,D=camera_matrix(profile)
    if profile['resolution']!=[front[0]['width'],front[0]['height']]:raise ValueError('Front lens resolution mismatch')
    trace=SpeedTrace(sei);t=np.array([f['camera_time_s'] for f in selected]);speed=np.interp(t,trace.times,trace.speeds)
    heading_rows=[r for r in sei if r['fields']['heading_deg']['valid']]
    if not heading_rows:raise ValueError('No ego heading telemetry')
    ht=np.array([r['camera_time_s'] for r in heading_rows]);angles=np.unwrap(np.radians([r['fields']['heading_deg']['value'] for r in heading_rows]));heading=np.interp(t,ht,angles)
    # Stationary heading noise cannot rotate the car. Raw signed speed is kept.
    moving=np.flatnonzero(abs(speed)>=.5);last=heading[moving[0]] if len(moving) else heading[0]
    for i in range(len(t)):
        if abs(speed[i])>=.5:last=heading[i]
        else:heading[i]=last
    yaw=-(heading-heading[0]);ego=np.zeros((len(t),3))
    for i in range(1,len(t)):
        distance=trace.distance(t[i-1],t[i]);angle=(yaw[i]+yaw[i-1])/2;ego[i]=ego[i-1]+distance*np.array([np.cos(angle),np.sin(angle),0])
    renderer=Renderer(ego_asset,actor_asset);rear_to_center=renderer.lengths[1]/2
    raw=[];support=[]
    for i,f in enumerate(selected):
        row=by_frame[f['frame_id']];b=row['bbox_xyxy'];ray=cv2.undistortPoints(np.array([[[(b[0]+b[2])/2,b[3]]]],float),K,D).reshape(2)
        if ray[1]<=.02:raise ValueError('Lead contact too close to assumed horizon')
        depth=camera_height_m/ray[1]
        if not 3<depth<80:raise ValueError('Lead depth outside prototype range')
        # The observed lower rear silhouette is a contact at the vehicle rear,
        # not its model centre. Keep this footprint offset explicit.
        raw.append(ego[i]+rotation(yaw[i])@np.array([depth+rear_to_center,-ray[0]*depth,0]));support.append(row['support'])
    raw=np.array(raw);fitted=smooth_positions(t,raw,strength=.2);velocity=np.gradient(fitted,t,axis=0);actor_yaw=[];last=yaw[0]
    for i,v in enumerate(velocity):
        if np.linalg.norm(v[:2])>1.5:last=math.atan2(v[1],v[0])
        actor_yaw.append(last)
    actor_yaw=np.unwrap(actor_yaw)
    work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False);video=work/'single-lead-3d.mp4';font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
    seq={c:sorted([f for f in frames if f['camera_id']==c],key=lambda f:f['camera_time_s']) for c in ['front','back','left_repeater','right_repeater']};cache={};payloads={};snapshots=[];fps=20;count=int(np.floor((t[-1]-t[0])*fps))+1
    with av.open(str(video),'w',options={'movflags':'+faststart'}) as output:
        stream=output.add_stream('libx264',rate=fps);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
        for j in range(count):
            now=t[0]+j/fps;ew=np.array([np.interp(now,t,ego[:,k]) for k in range(3)]);aw=np.array([np.interp(now,t,fitted[:,k]) for k in range(3)]);ey=np.interp(now,t,yaw);ay=np.interp(now,t,actor_yaw)
            relative=rotation(-ey)@(aw-ew);canvas=Image.new('RGB',(1280,1000),(22,27,31));canvas.paste(renderer.frame(relative,ay-ey),(0,0));draw=ImageDraw.Draw(canvas)
            draw.text((20,15),'SINGLE LEAD 3D PROTOTYPE / estimated ground geometry',font=font,fill=(50,58,64));draw.text((20,45),f'{(now-t[0]):05.2f}s | {int(round(np.interp(now,t,speed)*3.6))} km/h | full-interval trajectory fit',font=font,fill=(65,73,78))
            for k,(camera,sequence) in enumerate(seq.items()):
                f=min(sequence,key=lambda f:abs(f['camera_time_s']-now))
                if f['frame_id'] not in cache:cache[f['frame_id']]=Image.open(package/f['image']).convert('RGB').resize((320,240))
                tile=cache[f['frame_id']].copy();d=ImageDraw.Draw(tile);d.rectangle((0,0,320,24),fill=(15,20,24));d.text((4,2),f'{camera} {f["camera_time_s"]:.3f}s',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',14),fill='white');canvas.paste(tile,(k*320,720))
            draw.text((12,970),'Front lead only. Side/rear shown for comparison, not fused. Ground height/pitch are priors.',font=font,fill=(200,205,210))
            frame=av.VideoFrame.from_image(canvas);frame.pts=j;frame.time_base=Fraction(1,fps)
            for packet in stream.encode(frame):output.mux(packet)
            if j in [0,160,320,480,640]:snapshots.append(canvas.resize((640,500)))
            if j%200==0:print('Rendered single lead',j,'/',count,flush=True)
        for packet in stream.encode():output.mux(packet)
    with av.open(str(video)) as check:
        decoded=sum(1 for _ in check.decode(video=0));duration=check.duration/av.time_base
    if decoded!=count:raise ValueError('3D export decode count mismatch')
    sheet=Image.new('RGB',(1920,1000),'#eee')
    for i,image in enumerate(snapshots):sheet.paste(image,((i%3)*640,(i//3)*500))
    buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=95);payloads['proof/contact.jpg']=buf.getvalue()
    trajectory=[{'frame_id':f['frame_id'],'camera_time_s':float(t[i]),'ego_position_W':ego[i].tolist(),'ego_yaw_rad':float(yaw[i]),
        'raw_actor_position_W':raw[i].tolist(),'fitted_actor_position_W':fitted[i].tolist(),'actor_yaw_rad':float(actor_yaw[i]),'source_support':support[i],
        'metric_status':'estimated_from_ground_height_prior','uses_future_observations':True} for i,f in enumerate(selected)]
    report={'scope':'Single lead vehicle prototype only; no fused surrounding vehicles/lanes/rig acceptance','selected_local_track':key,
        'trajectory_samples':len(t),'duration_s':float(duration),'video_frames':count,'all_frames_decoded':True,'camera_height_m':camera_height_m,
        'camera_pitch_rad_assumed':0,'metric_accuracy':'NOT_EVALUATED','world_ego_source':'own front SEI signed speed and heading, independent of actors',
        'ego_pose_origin':'front_camera_horizontal_origin_assumed','ego_model_center_x_offset_m':-1.2,'lead_rear_to_center_m':rear_to_center,
        'trajectory_fit':'whole_interval_acceleration_regularization','actor_model_role':'explicit prototype asset visual class',
        'actor_model_kind':Path(actor_asset).stem.removeprefix('tesla_fsd_'),
        'renderer':renderer.ctx.info['GL_RENDERER'],'asset_axes':'GLB +Z front,+Y up -> W +X front,+Z up'}
    payloads.update({'single-lead-3d.mp4':video.read_bytes(),'trajectory.json':canonical(trajectory),'report.json':canonical(report)})
    return publish(destination,stage='single_lead_3d_prototype',payloads=payloads,
        inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','tracks','telemetry'],[pm,tm,sm])],
        config={'height_prior_m':camera_height_m,'pitch_prior_rad':0,'front_lens_candidate':profile,'offline_strength':.2,'fps':fps,'scope':'single_lead_only','ego_model_center_x_offset_m':-1.2,'lead_rear_to_center_m':rear_to_center},
        model_hashes=[file_hash(Path(ego_asset)),file_hash(Path(actor_asset))],code_revision=digest(Path(__file__).read_bytes()),
        runtime={'av':av.__version__,'render_device':renderer.ctx.info['GL_RENDERER'],'cuda':False})
