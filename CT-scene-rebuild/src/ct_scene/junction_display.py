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


def branch_edges(road):
    segments=[]
    for junction in road.get('junctions',[]):
        for arm in junction.get('branch_display_hints',[]):
            centre=np.asarray(arm['origin_W_xy']);angle=arm['heading_W_rad'];direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            station=float(coordinates(road,centre)[0]);bounds=display_bounds(road,station);tangent=float(coordinates(road,centre)[2]);sine=abs(math.sin(angle-tangent))
            if sine<.2:continue
            start=(bounds[0]-bounds[-1])/(2*sine)+arm['width_m']*.5+1.
            if start>=arm['length_m']:continue
            for sign in [-1,1]:
                offset=normal*arm['width_m']*.5*sign
                segments.append((centre+direction*start+offset,centre+direction*arm['length_m']+offset))
    return segments
