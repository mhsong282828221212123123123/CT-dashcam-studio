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


def is_turning_junction(junction):
    return abs(float(junction.get('turn_deg',0.)))>=30.


def _angle_diff(a,b):
    return float(np.arctan2(np.sin(a-b),np.cos(a-b)))


def turn_core_range(road,junction,deg_thresh=6.):
    """Locate the actual curved turning core inside a padded turning junction range."""
    a,b=map(float,junction['road_station_range_m'])
    if b-a<=18.:return a,b
    t_in=float(coordinates(road,world(road,a,0))[2])
    t_out=float(coordinates(road,world(road,b,0))[2])
    samples=np.linspace(a,b,max(16,int((b-a)/1.5)+1))
    headings=np.array([float(coordinates(road,world(road,float(s),0))[2]) for s in samples])
    thresh=math.radians(deg_thresh)
    dev_in=np.abs(np.arctan2(np.sin(headings-t_in),np.cos(headings-t_in)))>=thresh
    dev_out=np.abs(np.arctan2(np.sin(headings-t_out),np.cos(headings-t_out)))>=thresh
    s_in=float(samples[np.argmax(dev_in)]) if np.any(dev_in) else a
    s_out=float(samples[len(samples)-1-np.argmax(dev_out[::-1])]) if np.any(dev_out) else b
    s_in=max(a,s_in-1.5);s_out=min(b,s_out+1.5)
    if s_out-s_in<12.:
        mid=(a+b)/2;return max(a,mid-8.),min(b,mid+8.)
    return s_in,s_out


def opposing_enabled_at(road,station=None):
    opp=road.get('opposing_display') or {}
    if not opp.get('enabled'):return False
    if station is None:return True
    sec=section_at(road,station)
    if sec is not None and 'opposing_display' in sec:
        return bool(sec['opposing_display'])
    sections=road.get('lane_sections',[])
    if not sections:return True
    before=[s for s in sections if s['station_range_m'][1]<=station]
    after=[s for s in sections if s['station_range_m'][0]>=station]
    if before and after:
        sa=max(before,key=lambda s:s['station_range_m'][1])
        sb=min(after,key=lambda s:s['station_range_m'][0])
        mid=(sa['station_range_m'][1]+sb['station_range_m'][0])/2
        chosen=sa if station<mid else sb
        if 'opposing_display' in chosen:return bool(chosen['opposing_display'])
    elif before and 'opposing_display' in before[-1]:
        return bool(max(before,key=lambda s:s['station_range_m'][1])['opposing_display'])
    elif after and 'opposing_display' in after[0]:
        return bool(min(after,key=lambda s:s['station_range_m'][0])['opposing_display'])
    return True


def opposing_width(road,station=None):
    if not opposing_enabled_at(road,station):return 0.
    b=display_bounds(road,station) if station is not None else road.get('boundaries_y',[3.5,0.])
    return max(3.0,float(b[0]-b[-1]))


def turn_apex(road,junction,full_carriageway=False):
    """2D intersection of straight entry and exit rays for a turning junction."""
    a,b=map(float,junction['road_station_range_m'])
    ba=display_bounds(road,a-.5);bb=display_bounds(road,b+.5)
    ow_in=opposing_width(road,a-.5) if full_carriageway else 0.
    ow_out=opposing_width(road,b+.5) if full_carriageway else 0.
    p_in=np.asarray(world(road,a,(ba[0]+ow_in+ba[-1])/2),float)
    t_in=float(coordinates(road,world(road,a,0))[2])
    p_out=np.asarray(world(road,b,(bb[0]+ow_out+bb[-1])/2),float)
    t_out=float(coordinates(road,world(road,b,0))[2])
    d_in=np.array([math.cos(t_in),math.sin(t_in)])
    d_out=np.array([math.cos(t_out),math.sin(t_out)])
    M=np.column_stack([d_in,-d_out])
    max_dist=max(60.,(b-a)*1.2)
    if abs(float(np.linalg.det(M)))>.15:
        u_in,u_out=np.linalg.solve(M,p_out-p_in)
        if 0.<=u_in<=max_dist and -max_dist<=u_out<=0.:
            return p_in+u_in*d_in
    mid=(a+b)/2;bm=display_bounds(road,mid)
    ow_m=opposing_width(road,mid) if full_carriageway else 0.
    return np.asarray(world(road,mid,(bm[0]+ow_m+bm[-1])/2),float)


def straight_box_half(road,junction,station=None):
    if station is None:
        peaks=junction.get('camera_station_peaks',[])
        station=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
    tangent=float(coordinates(road,world(road,station,0))[2])
    arms=junction.get('branch_display_hints',[])
    halves=[float(a.get('width_m',5.))/(2*abs(math.sin(float(a['heading_W_rad'])-tangent)))
            for a in arms if abs(math.sin(float(a['heading_W_rad'])-tangent))>=.2]
    if halves:return float(np.clip(max(halves),3.5,9.))
    bounds=display_bounds(road,station)
    return float(np.clip((bounds[0]-bounds[-1])*.5+1.5,4.,8.))


def in_intersection_box(road,station):
    """True when station sits inside the actual intersection core (not padded gap)."""
    for junction in road.get('junctions',[]):
        if is_turning_junction(junction):
            s_in,s_out=turn_core_range(road,junction)
            if s_in-2.<=station<=s_out+2.:return True
        else:
            peaks=junction.get('camera_station_peaks',[])
            arms=junction.get('branch_display_hints',[])
            if not peaks and not arms:continue
            centre=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
            half=straight_box_half(road,junction,centre)
            if abs(station-centre)<=half+3.2:return True
    return False


def _turn_arm_geometry(road,junction,arm):
    """Match turning-junction branch arm width and origin to the intersecting carriageways."""
    a,b=map(float,junction['road_station_range_m'])
    ba,bb=display_bounds(road,a-.5),display_bounds(road,b+.5)
    w_in=(ba[0]-ba[-1])+opposing_width(road,a-.5)
    w_out=(bb[0]-bb[-1])+opposing_width(road,b+.5)
    t_in=float(coordinates(road,world(road,a,0))[2])
    t_out=float(coordinates(road,world(road,b,0))[2])
    angle=float(arm['heading_W_rad'])
    apex=turn_apex(road,junction,full_carriageway=True)
    align_in=abs(_angle_diff(angle,t_in))
    align_out=abs(_angle_diff(angle,t_out+math.pi))
    if align_in<=align_out and align_in<math.radians(40):
        arm_w=float(np.clip(w_in,5.5,14.));cross_half=float(np.clip(w_out*.5,3.5,8.))
    elif align_out<math.radians(40):
        arm_w=float(np.clip(w_out,5.5,14.));cross_half=float(np.clip(w_in*.5,3.5,8.))
    else:
        arm_w=float(np.clip(max(float(arm.get('width_m',5.5)),min(w_in,w_out)),5.5,12.))
        cross_half=float(np.clip(max(w_in,w_out)*.5,4.,8.))
    return apex,arm_w,cross_half


def mouth_edges(road,station):
    """Retain the closed curb at a T; open only a supported branch side."""
    left=right=True
    for junction in road.get('junctions',[]):
        if is_turning_junction(junction):
            s_in,s_out=turn_core_range(road,junction)
            if s_in<=station<=s_out:
                return False,False
            continue
        peaks=junction.get('camera_station_peaks',[])
        centre=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
        tangent=float(coordinates(road,world(road,centre,0))[2])
        for arm in junction.get('branch_display_hints',[]):
            delta=arm['heading_W_rad']-tangent;side=math.sin(delta)
            if abs(side)<.2:continue
            half=min(10.,arm['width_m']/(2*abs(side))+.15)
            if abs(station-centre)>half:continue
            if side>0:left=False
            else:right=False
    return left,right


def branch_display_polygons(road):
    """Asphalt polygons for branch arms (matching carriageway width on turning junctions)."""
    polys=[]
    for junction in road.get('junctions',[]):
        is_turn=is_turning_junction(junction)
        for arm in junction.get('branch_display_hints',[]):
            angle=float(arm['heading_W_rad']);direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            length=float(arm.get('length_m',25.))
            if is_turn:
                centre,arm_w,cross_half=_turn_arm_geometry(road,junction,arm)
                s0=-cross_half;s1=max(length,cross_half+18.)
                polys.append([(centre+direction*s+normal*d).tolist() for s,d in [(s0,-arm_w/2),(s1,-arm_w/2),(s1,arm_w/2),(s0,arm_w/2)]])
            else:
                polys.append(arm['polygon_W_xy'])
    return polys


def branch_edges(road):
    segments=[]
    for junction in road.get('junctions',[]):
        is_turn=is_turning_junction(junction)
        for arm in junction.get('branch_display_hints',[]):
            angle=float(arm['heading_W_rad']);direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            length=float(arm.get('length_m',25.))
            if is_turn:
                centre,arm_w,cross_half=_turn_arm_geometry(road,junction,arm)
                start=cross_half+.5;end_s=max(length,cross_half+18.)
            else:
                centre=np.asarray(arm['origin_W_xy'],float);arm_w=float(arm['width_m'])
                station=float(coordinates(road,centre)[0]);bounds=display_bounds(road,station);tangent=float(coordinates(road,centre)[2]);sine=abs(math.sin(angle-tangent))
                if sine<.2:continue
                opp_w=opposing_width(road,station)
                is_left=math.sin(angle-tangent)>0
                extra=opp_w if is_left else 0.
                # Flush with the outer curb (+0.05m so start sits cleanly outside main curb)
                start=((bounds[0]-bounds[-1])/2.+extra)/sine+.05
                end_s=length
            if start>=end_s:continue
            for sign in [-1,1]:
                offset=normal*arm_w*.5*sign
                segments.append((centre+direction*start+offset,centre+direction*end_s+offset))
    return segments


def _add_crosswalk(road,cw_s,crosswalks):
    b_cw=display_bounds(road,cw_s);ow_cw=opposing_width(road,cw_s)
    d_cur=b_cw[-1]+.45
    while d_cur+.45<=b_cw[0]+ow_cw-.35:
        c1,c2=world(road,cw_s-1.2,d_cur),world(road,cw_s+1.2,d_cur)
        c3,c4=world(road,cw_s+1.2,d_cur+.45),world(road,cw_s-1.2,d_cur+.45)
        crosswalks.append([c1.tolist(),c2.tolist(),c3.tolist(),c4.tolist()])
        d_cur+=.95


def _add_stop_bar(road,sb_s,sign,stop_bars):
    b_sb=display_bounds(road,sb_s);ow_sb=opposing_width(road,sb_s)
    if sign==1 and ow_sb<=0:return
    sb_hi,sb_lo=(b_sb[0]-.15,b_sb[-1]+.15) if sign==-1 else (b_sb[0]+ow_sb-.15,b_sb[0]+.15)
    p_l1,p_r1=world(road,sb_s-.20,sb_hi),world(road,sb_s-.20,sb_lo)
    p_l2,p_r2=world(road,sb_s+.20,sb_hi),world(road,sb_s+.20,sb_lo)
    stop_bars.append([p_l1.tolist(),p_r1.tolist(),p_r2.tolist(),p_l2.tolist()])


def _convex_hull_2d(pts):
    pts=sorted({(round(float(p[0]),3),round(float(p[1]),3)) for p in pts})
    if len(pts)<=2:return [list(p) for p in pts]
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[]
    for p in pts:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(pts):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1]+upper[:-1]]


def junction_markings(road):
    """Procedural intersection aprons, stop bars, crosswalk zebra stripes, and branch centerlines.

    Pure display geometry derived from existing camera/OSM junction hints; never
    alters vehicle coordinates, lane scopes, or collision constraints.
    """
    hubs=[];stop_bars=[];crosswalks=[];branch_centers=[]
    extent=road.get('reference_line',{}).get('supported_extent_m')
    for junction in road.get('junctions',[]):
        arms=junction.get('branch_display_hints',[])
        peaks=junction.get('camera_station_peaks',[])
        if not arms and not peaks:continue
        if is_turning_junction(junction):
            s_in,s_out=turn_core_range(road,junction)
            if arms:
                # Build a clean solid intersection apron connecting straight entry/exit mouths
                # and the full-carriageway apex (never a detached floating circle).
                ba,bb=display_bounds(road,s_in),display_bounds(road,s_out)
                ow_in,ow_out=opposing_width(road,s_in),opposing_width(road,s_out)
                apex=turn_apex(road,junction,full_carriageway=True)
                apron_pts=[
                    world(road,s_in,ba[-1]),world(road,s_in,ba[0]+ow_in),
                    world(road,s_out,bb[-1]),world(road,s_out,bb[0]+ow_out),
                    apex,
                ]
                for arm in arms:
                    orig,arm_w,cross_half=_turn_arm_geometry(road,junction,arm)
                    ang=float(arm['heading_W_rad']);dr=np.array([math.cos(ang),math.sin(ang)]);nm=np.array([-dr[1],dr[0]])
                    apron_pts.append(orig+dr*cross_half-nm*(arm_w*.5))
                    apron_pts.append(orig+dr*cross_half+nm*(arm_w*.5))
                hull=_convex_hull_2d(apron_pts)
                if len(hull)>=3:hubs.append(hull)
            # Approach/departure crosswalks ONLY at the straight turn entry and exit mouths.
            for sign,cw_s,sb_s in [(-1,s_in-1.6,s_in-3.4),(1,s_out+1.6,s_out+3.4)]:
                if extent and not (extent[0]+2.<=sb_s<=extent[1]-2.):continue
                _add_stop_bar(road,sb_s,sign,stop_bars)
                _add_crosswalk(road,cw_s,crosswalks)
            for arm in arms:
                orig,arm_w,cross_half=_turn_arm_geometry(road,junction,arm)
                angle=float(arm['heading_W_rad'])
                direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
                arm_len=max(float(arm.get('length_m',25.)),cross_half+18.)
                start=cross_half+.5
                if start+2.>=arm_len:continue
                if arm_w>=5.0:
                    s_cur=start+3.0
                    while s_cur+2.2<=arm_len:
                        s_end=min(s_cur+2.6,arm_len)
                        p0,p1=orig+direction*s_cur,orig+direction*s_end
                        branch_centers.append([(p0-normal*.06).tolist(),(p1-normal*.06).tolist(),(p1+normal*.06).tolist(),(p0+normal*.06).tolist()])
                        s_cur+=5.0
                cw_orig=orig+direction*(start+1.4)
                d_cur=-arm_w*.43
                while d_cur+.42<=arm_w*.43:
                    p0=cw_orig+normal*d_cur;p1=cw_orig+normal*(d_cur+.42)
                    crosswalks.append([(p0-direction*1.1).tolist(),(p0+direction*1.1).tolist(),(p1+direction*1.1).tolist(),(p1-direction*1.1).tolist()])
                    d_cur+=.90
            continue
        station=float(np.median([p['s'] for p in peaks])) if peaks else sum(junction['road_station_range_m'])/2
        bounds=display_bounds(road,station)
        opp_w=opposing_width(road,station)
        full_left=bounds[0]+opp_w;full_right=bounds[-1]
        box_half=straight_box_half(road,junction,station)
        tangent=float(coordinates(road,world(road,station,0))[2])
        has_left_arm=any(math.sin(float(a['heading_W_rad'])-tangent)>.2 for a in arms)
        has_right_arm=any(math.sin(float(a['heading_W_rad'])-tangent)<-.2 for a in arms)
        if arms and (has_left_arm and has_right_arm):
            # Clean rectangular intersection box instead of an oversized round blob
            hubs.append([world(road,s,d).tolist() for s,d in [
                (station-box_half,full_right),(station+box_half,full_right),
                (station+box_half,full_left),(station-box_half,full_left)]])
        # Approach stop bar (own lanes only) + zebra crosswalk (full symmetric width), clear of cross-arm
        for sign in (-1,1):
            cw_s=station+sign*(box_half+1.8)
            sb_s=station+sign*(box_half+3.6)
            if extent and not (extent[0]+2.<=sb_s<=extent[1]-2.):continue
            b_sb=display_bounds(road,sb_s);ow_sb=opposing_width(road,sb_s)
            sb_hi,sb_lo=(b_sb[0]-.15,b_sb[-1]+.15) if sign==-1 or ow_sb<=0 else (b_sb[0]+ow_sb-.15,b_sb[0]+.15)
            p_l1,p_r1=world(road,sb_s-.20,sb_hi),world(road,sb_s-.20,sb_lo)
            p_l2,p_r2=world(road,sb_s+.20,sb_hi),world(road,sb_s+.20,sb_lo)
            stop_bars.append([p_l1.tolist(),p_r1.tolist(),p_r2.tolist(),p_l2.tolist()])
            _add_crosswalk(road,cw_s,crosswalks)
        # Branch arm yellow centerlines + branch entry crosswalks (flush outside main curb)
        for arm in arms:
            orig=np.asarray(arm['origin_W_xy'],float);angle=float(arm['heading_W_rad'])
            direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            sine=abs(math.sin(angle-tangent))
            if sine<.2:continue
            is_left=math.sin(angle-tangent)>0
            arm_w=float(arm.get('width_m',5.));arm_len=float(arm.get('length_m',25.))
            start=((bounds[0]-bounds[-1])/2.+(opp_w if is_left else 0.))/sine+.05
            if start+2.>=arm_len:continue
            if arm_w>=4.2:
                s_cur=start+3.2
                while s_cur+2.2<=arm_len:
                    s_end=min(s_cur+2.6,arm_len)
                    p0,p1=orig+direction*s_cur,orig+direction*s_end
                    branch_centers.append([(p0-normal*.06).tolist(),(p1-normal*.06).tolist(),(p1+normal*.06).tolist(),(p0+normal*.06).tolist()])
                    s_cur+=5.0
            cw_orig=orig+direction*(start+1.5)
            d_cur=-arm_w*.42
            while d_cur+.4<=arm_w*.42:
                p0=cw_orig+normal*d_cur;p1=cw_orig+normal*(d_cur+.4)
                crosswalks.append([(p0-direction*1.0).tolist(),(p0+direction*1.0).tolist(),(p1+direction*1.0).tolist(),(p1-direction*1.0).tolist()])
                d_cur+=.85
    return dict(hubs=hubs,stop_bars=stop_bars,crosswalks=crosswalks,branch_centers=branch_centers)
