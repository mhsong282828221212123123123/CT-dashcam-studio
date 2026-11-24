"""Display-only asphalt evidence; independent of lane IDs and actor positions."""
from collections import defaultdict
import json
from pathlib import Path
import cv2
import numpy as np
from .affine_probe import camera_matrix
from .prototype_front import rotation


def frame_cells(mask,boxes,profile,mount,ego_position,ego_yaw,cell_m=.5,with_support=False):
    """Project repeated near-field ground evidence, excluding dynamic bodies."""
    h,w=mask.shape;allowed=(mask>0).astype('uint8')
    allowed[:round(h*.53)]=0;allowed[round(h*.92):]=0
    allowed[:,:round(w*.03)]=0;allowed[:,round(w*.97):]=0
    observed=allowed.copy();inferred=np.zeros_like(allowed)
    for box in boxes:
        x,y,X,Y=np.rint(box).astype(int)
        allowed[max(0,y-3):min(h,Y+4),max(0,x-3):min(w,X+4)]=0
        if x<16 or X>w-16:continue
        lo=max(y,round(h*.53));hi=min(Y,h)
        if hi<=lo:continue
        left=np.mean(observed[lo:hi,x-16:x-4]>0,axis=1)>.8
        right=np.mean(observed[lo:hi,X+4:X+16]>0,axis=1)>.8
        for row in np.flatnonzero(left&right)+lo:
            allowed[row,x:X]=1;inferred[row,x:X]=1
    K,D=camera_matrix(profile);height=float(mount['height_m'])
    yaw=ego_yaw+np.radians(mount['yaw_deg']);R=rotation(yaw)
    eye=np.asarray(ego_position)+rotation(ego_yaw)@np.asarray(mount['offset_W_ego_m'])
    x,y=np.meshgrid(np.arange(4.,40.,cell_m),np.arange(-18.,18.,cell_m))
    local=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    seeds=(local@R.T+eye)[:,:2]
    keys=np.unique(np.floor(seeds/cell_m).astype(int),axis=0)
    centres=(keys+.5)*cell_m
    offsets=np.array([[0,0],[-.42,-.42],[-.42,.42],[.42,-.42],[.42,.42]])*cell_m
    points=np.c_[np.repeat(centres,5,axis=0)+np.tile(offsets,(len(keys),1)),np.zeros(len(keys)*5)]
    relative=(points-eye)@R
    camera=np.c_[-relative[:,1],np.full(len(points),height),relative[:,0]]
    uv=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),K,D)[0][:,0,:]
    uv=np.rint(uv).astype(int)
    valid=(relative[:,0]>=3.5)&(relative[:,0]<=40.5)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    support=np.zeros(len(points),bool)
    support[valid]=allowed[uv[valid,1],uv[valid,0]]>0
    accepted=np.all(support.reshape(-1,5),axis=1)
    if with_support:
        hidden=np.zeros(len(points),bool);hidden[valid]=inferred[uv[valid,1],uv[valid,0]]>0
        return keys[accepted],~np.any(hidden.reshape(-1,5),axis=1)[accepted]
    return keys[accepted]


def rectangles(keys,cell_m):
    by_y=defaultdict(list)
    for x,y in keys:by_y[int(y)].append(int(x))
    result=[]
    for y,xs in sorted(by_y.items()):
        xs=sorted(set(xs));start=last=xs[0]
        for x in xs[1:]+[xs[-1]+2]:
            if x!=last+1:
                result.append([start*cell_m,y*cell_m,(last+1)*cell_m,(y+1)*cell_m]);start=x
            last=x
    return result


def surface_mesh(keys,cell_m):
    if len(keys)==0:return dict(vertices_W_xy=[],triangles_indices=[])
    keys=np.asarray(keys,int);origin=keys.min(axis=0)-1;shape=keys.max(axis=0)-origin+2
    raster=np.zeros(tuple(shape[::-1]),np.uint8);q=keys-origin;raster[q[:,1],q[:,0]]=1
    count,labels,stats,_=cv2.connectedComponentsWithStats(raster,8)
    selected=[]
    for label in range(1,count):
        if stats[label,cv2.CC_STAT_AREA]*cell_m**2<3.:continue
        component=(labels==label).astype('uint8')
        contours,hierarchy=cv2.findContours(component,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for i,r in enumerate(hierarchy[0]):
            if r[3]>=0 and cv2.contourArea(contours[i])*cell_m**2<3.:
                cv2.drawContours(component,contours,i,1,cv2.FILLED)
        yy,xx=np.where(component>0);selected.extend(map(tuple,np.c_[xx,yy]+origin))
    vertices={};edges=defaultdict(int);quads=[]
    for x,y in selected:
        quad=[(x,y),(x+1,y),(x+1,y+1),(x,y+1)];quads.append(quad)
        for v in quad:vertices.setdefault(v,len(vertices))
        for a,b in zip(quad,quad[1:]+quad[:1]):edges[tuple(sorted((a,b)))]+=1
    adjacent=defaultdict(set)
    for (a,b),count in edges.items():
        if count==1:adjacent[a].add(b);adjacent[b].add(a)
    positions={v:np.array(v,float)*cell_m for v in vertices}
    for _ in range(4):
        smooth=dict(positions)
        for v,neighbors in adjacent.items():
            candidate=.5*positions[v]+.5*np.mean([positions[n] for n in neighbors],axis=0)
            origin_v=np.array(v,float)*cell_m;delta=candidate-origin_v;length=np.linalg.norm(delta)
            smooth[v]=origin_v+delta*min(1.,cell_m*.5/max(length,1e-9))
        positions=smooth
    triangles=[]
    for quad in quads:
        a,b,c,d=[vertices[v] for v in quad];triangles.extend([[a,b,c],[a,c,d]])
    return dict(vertices_W_xy=[positions[v].tolist() for v in vertices],triangles_indices=triangles)


def extract(road_masks,frames,observations,times,ego_positions,ego_yaws,profile,mount,cell_m=.5):
    records={r['frame_id']:r for r in json.loads((Path(road_masks)/'frames.json').read_text())}
    evidence=defaultdict(list);used=0
    for f in frames:
        if f['camera_id']!='front' or f['frame_id'] not in records:continue
        r=records[f['frame_id']];mask=cv2.imread(str(Path(road_masks)/r['road']),cv2.IMREAD_GRAYSCALE)
        if mask is None or mask.shape!=(f['height'],f['width']):raise ValueError('Road surface mask/source shape mismatch')
        t=f['camera_time_s'];eye=[np.interp(t,times,ego_positions[:,k]) for k in range(3)]
        yaw=float(np.interp(t,times,np.unwrap(ego_yaws)))
        keys,direct=frame_cells(mask,[o['bbox_xyxy'] for o in observations.get(f['frame_id'],[])],profile,mount,eye,yaw,cell_m,with_support=True)
        for key,seen in zip(keys,direct):evidence[tuple(map(int,key))].append((t,f['frame_id'],bool(seen)))
        used+=1
    cells=[]
    for (x,y),rows in sorted(evidence.items()):
        distinct={fid for t,fid,seen in rows};ts=[t for t,fid,seen in rows]
        if len(distinct)<3 or max(ts)-min(ts)<.399:continue
        cells.append(dict(grid_xy=[x,y],frames=len(distinct),time_range_s=[min(ts),max(ts)],
                          directly_observed_frames=sum(seen for t,fid,seen in rows),
                          occlusion_continuity_frames=sum(not seen for t,fid,seen in rows),
                          evidence_frame_ids=list(dict.fromkeys(fid for t,fid,seen in rows))[:3]))
    mesh=surface_mesh([r['grid_xy'] for r in cells],cell_m)
    return dict(cell_m=cell_m,cells=cells,mesh=mesh,
                source='repeated_front_road_masks_with_bounded_occlusion_continuity',frames_used=used,
                display_boundary_completion='small_enclosed_mask_holes_below_3_m2; smoothed_outline',
                use='display_only; no_lane_count_scope_identity_or_vehicle_pose_input',
                geometry='estimated_ground_plane; no_3D_road_or_height_claim')
