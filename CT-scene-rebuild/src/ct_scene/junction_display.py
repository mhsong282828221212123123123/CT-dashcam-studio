"""Display a continuous camera corridor through uncertain junction scopes.

These inferred surfaces never assign a lane, move a car or register GPS.
"""
import math
import numpy as np
from .road_geometry import boundaries_at,section_at
from .road_frame import world,coordinates


def display_bounds(road,station):
    if section_at(road,station):return boundaries_at(road,station)
    sections=road.get('lane_sections',[])
    before=[s for s in sections if s['station_range_m'][1]<=station]
    after=[s for s in sections if s['station_range_m'][0]>=station]
    if not before or not after:return boundaries_at(road,station)
    a=max(before,key=lambda s:s['station_range_m'][1]);b=min(after,key=lambda s:s['station_range_m'][0]);lo=a['station_range_m'][1];hi=b['station_range_m'][0]
    w=(station-lo)/max(.001,hi-lo);left=(1-w)*a['boundaries_y'][0]+w*b['boundaries_y'][0];right=(1-w)*a['boundaries_y'][-1]+w*b['boundaries_y'][-1]
    return [float(left),float(right)]


def corridor_gaps(road):
    sections=sorted(road.get('lane_sections',[]),key=lambda s:s['station_range_m'][0]);result=[]
    for a,b in zip(sections,sections[1:]):
        lo,hi=a['station_range_m'][1],b['station_range_m'][0]
        if hi-lo<.5:continue
        if any(max(lo,j['road_station_range_m'][0])<min(hi,j['road_station_range_m'][1]) for j in road.get('junctions',[])):
            result.append((float(lo),float(hi)))
    return result


def mouth_edges(road,station):
    """Retain the closed curb at a T; open only a supported branch side."""
    left=right=True
    for junction in road.get('junctions',[]):
        peaks=junction.get('camera_station_peaks',[])
        centre=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
        tangent=float(coordinates(road,world(road,centre,0))[2])
        for arm in junction.get('branch_display_hints',[]):
            delta=arm['heading_W_rad']-tangent;side=math.sin(delta)
            if abs(side)<.2:continue
            half=min(10.,arm['width_m']/(2*abs(side))+2.)
            if abs(station-centre)>half:continue
            if side>0:left=False
            else:right=False
    return left,right


def opposing_width(road,station=None):
    opp=road.get('opposing_display') or {}
    if not opp.get('enabled'):return 0.
    max_same=opp.get('max_same_dir_lanes')
    b=display_bounds(road,station) if station is not None else road.get('boundaries_y',[3.5,0.])
    if max_same is not None and road.get('left_boundary_role')=='inferred_extent_not_confirmed_centreline' and len(b)-1>max_same:
        return 0.
    return max(3.0,float(b[0]-b[-1]))


def branch_edges(road):
    segments=[]
    for junction in road.get('junctions',[]):
        for arm in junction.get('branch_display_hints',[]):
            centre=np.asarray(arm['origin_W_xy']);angle=arm['heading_W_rad'];direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            station=float(coordinates(road,centre)[0]);bounds=display_bounds(road,station);tangent=float(coordinates(road,centre)[2]);sine=abs(math.sin(angle-tangent))
            if sine<.2:continue
            opp_w=opposing_width(road,station)
            is_left=math.sin(angle-tangent)>0
            extra=opp_w if is_left else 0.
            start=(bounds[0]-bounds[-1]+2*extra)/(2*sine)+arm['width_m']*.5+1.
            if start>=arm['length_m']:continue
            for sign in [-1,1]:
                offset=normal*arm['width_m']*.5*sign
                segments.append((centre+direction*start+offset,centre+direction*arm['length_m']+offset))
    return segments


def junction_markings(road):
    """Procedural intersection hubs, stop bars, crosswalk zebra stripes, and branch centerlines.

    Pure display geometry derived from existing camera/OSM junction hints; never
    alters vehicle coordinates, lane scopes, or collision constraints.
    """
    hubs=[];stop_bars=[];crosswalks=[];branch_centers=[]
    extent=road.get('reference_line',{}).get('supported_extent_m')
    for junction in road.get('junctions',[]):
        arms=junction.get('branch_display_hints',[])
        peaks=junction.get('camera_station_peaks',[])
        if not arms and not peaks:continue
        station=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
        bounds=display_bounds(road,station)
        opp_w=opposing_width(road,station)
        full_left=bounds[0]+opp_w;full_right=bounds[-1]
        corridor_w=max(3.5,full_left-full_right)
        half=min(9.,max(4.,(bounds[0]-bounds[-1])*.5+1.5))
        tangent=float(coordinates(road,world(road,station,0))[2])
        has_left_arm=any(math.sin(float(a['heading_W_rad'])-tangent)>.2 for a in arms)
        has_right_arm=any(math.sin(float(a['heading_W_rad'])-tangent)<-.2 for a in arms)
        if arms and (has_left_arm and has_right_arm):
            centre=np.asarray(world(road,station,(full_left+full_right)/2),float)
            max_arm_w=max(float(a.get('width_m',5.)) for a in arms)
            radius=min(12.,max(corridor_w*.52+.8,max_arm_w*.75+1.0))
            angles=np.linspace(0,2*math.pi,16,endpoint=False)
            hubs.append((centre+np.c_[np.cos(angles),np.sin(angles)]*radius).tolist())
        # Approach stop bar (own lanes only) + zebra crosswalk (full symmetric two-way width)
        for sign in (-1,1):
            cw_s=station+sign*(half+1.6)
            sb_s=station+sign*(half+3.4)
            if extent and not (extent[0]+2.<=sb_s<=extent[1]-2.):continue
            b_sb=display_bounds(road,sb_s);ow_sb=opposing_width(road,sb_s)
            sb_hi,sb_lo=(b_sb[0]-.15,b_sb[-1]+.15) if sign==-1 or ow_sb<=0 else (b_sb[0]+ow_sb-.15,b_sb[0]+.15)
            p_l1,p_r1=world(road,sb_s-.20,sb_hi),world(road,sb_s-.20,sb_lo)
            p_l2,p_r2=world(road,sb_s+.20,sb_hi),world(road,sb_s+.20,sb_lo)
            stop_bars.append([p_l1.tolist(),p_r1.tolist(),p_r2.tolist(),p_l2.tolist()])
            b_cw=display_bounds(road,cw_s);ow_cw=opposing_width(road,cw_s)
            d_cur=b_cw[-1]+.45
            while d_cur+.45<=b_cw[0]+ow_cw-.35:
                c1,c2=world(road,cw_s-1.2,d_cur),world(road,cw_s+1.2,d_cur)
                c3,c4=world(road,cw_s+1.2,d_cur+.45),world(road,cw_s-1.2,d_cur+.45)
                crosswalks.append([c1.tolist(),c2.tolist(),c3.tolist(),c4.tolist()])
                d_cur+=.95
        # Branch arm yellow centerlines + branch entry crosswalks
        for arm in arms:
            orig=np.asarray(arm['origin_W_xy'],float);angle=float(arm['heading_W_rad'])
            direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            sine=abs(math.sin(angle-tangent))
            if sine<.2:continue
            is_left=math.sin(angle-tangent)>0
            arm_w=float(arm.get('width_m',5.));arm_len=float(arm.get('length_m',25.))
            start=(bounds[0]-bounds[-1]+(2*opp_w if is_left else 0.))/(2*sine)+arm_w*.45+1.
            if start+1.5>=arm_len:continue
            if arm_w>=4.2:
                s_cur=start+2.5
                while s_cur+2.2<=arm_len:
                    s_end=min(s_cur+2.6,arm_len)
                    p0,p1=orig+direction*s_cur,orig+direction*s_end
                    branch_centers.append([(p0-normal*.06).tolist(),(p1-normal*.06).tolist(),(p1+normal*.06).tolist(),(p0+normal*.06).tolist()])
                    s_cur+=5.0
            cw_orig=orig+direction*(start+1.1)
            d_cur=-arm_w*.42
            while d_cur+.4<=arm_w*.42:
                p0=cw_orig+normal*d_cur;p1=cw_orig+normal*(d_cur+.4)
                crosswalks.append([(p0-direction*1.0).tolist(),(p0+direction*1.0).tolist(),(p1+direction*1.0).tolist(),(p1-direction*1.0).tolist()])
                d_cur+=.85
    return dict(hubs=hubs,stop_bars=stop_bars,crosswalks=crosswalks,branch_centers=branch_centers)
