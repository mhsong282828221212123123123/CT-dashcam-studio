"""Local road extent with fixed main-lane IDs and an optional left pocket -1."""
import numpy as np


def section_at(road,station):
    sections=road.get('lane_sections',[])
    section=next((s for s in sections if s['station_range_m'][0]<=station<s['station_range_m'][1]),None)
    if section and section.get('boundary_profile') and section.get('left_added_lane'):
        values=profile_boundaries(section,station)
        section=dict(section,id=section['id']+(':left-open' if values[0]-values[1]>=section.get('minimum_profile_lane_width_m',.8) else ':left-closed'))
    if section and section.get('camera_width_profile'):
        values=profile_boundaries(section,station);count=sum(a-b>=section.get('minimum_profile_lane_width_m',.8) for a,b in zip(values,values[1:]))
        section=dict(section,id=section['id']+':layout-'+str(count))
    return section


def profile_boundaries(section,station):
    knots=section['boundary_profile'];times=[k['station_m'] for k in knots]
    return [float(np.interp(station,times,[k['boundaries_y'][i] for k in knots])) for i in range(len(knots[0]['boundaries_y']))]


def boundaries_at(road,station):
    section=section_at(road,station)
    if section and section.get('boundary_profile'):
        values=profile_boundaries(section,station);minimum=section.get('minimum_profile_lane_width_m',.8)
        while len(values)>2 and values[0]-values[1]<minimum:values.pop(0)
        while len(values)>2 and values[-2]-values[-1]<minimum:values.pop(-2)
        return values
    if section:
        return section['boundaries_y']
    sections=road.get('lane_sections',[])
    if len(sections)>=2:
        prev_s=max((s for s in sections if s['station_range_m'][1]<=station),key=lambda s:s['station_range_m'][1],default=None)
        next_s=min((s for s in sections if s['station_range_m'][0]>station),key=lambda s:s['station_range_m'][0],default=None)
        if prev_s and next_s and len(prev_s['boundaries_y'])==len(next_s['boundaries_y']):
            span=next_s['station_range_m'][0]-prev_s['station_range_m'][1]
            t=float(np.clip((station-prev_s['station_range_m'][1])/max(span,1e-6),0.,1.))
            return [float((1.-t)*a+t*b) for a,b in zip(prev_s['boundaries_y'],next_s['boundaries_y'])]
    return road['boundaries_y']


def support_kind(road,station):
    if not supported(road,station):return 'unknown'
    for r in road.get('reference_line',{}).get('inferred_reference_intervals_m',[]):
        if r['station_range_m'][0]<=station<=r['station_range_m'][1]:return 'two_sided_camera_edges_inferred_reference'
    return 'camera_reference_or_explicit_geometry_prior'


def supported(road,station):
    if any(j['road_station_range_m'][0]<station<j['road_station_range_m'][1] for j in road.get('junctions',[])):return False
    if road.get('lane_sections'):
        section=section_at(road,station)
        if section is None or not section.get('supported',True):return False
    line=road.get('reference_line')
    if not line:return True
    extent=line.get('supported_extent_m')
    if extent and not extent[0]<=station<=extent[1]:return False
    return not any(a<station<b for a,b in line.get('unsupported_intervals_m',[]))


def pocket_width(pockets,x):
    return max([float(np.interp(x,[k[0] for k in p['width_knots']],[k[1] for k in p['width_knots']],left=0,right=0)) for p in pockets],default=0.)


def lane_bounds(road,x,lane):
    b=boundaries_at(road,x)
    if lane==-1:return b[0]+pocket_width(road.get('left_pockets',[]),x),b[0]
    return b[lane],b[lane+1]


def lane_at(road,x,lateral,margin=0):
    if not supported(road,x):return None
    b=boundaries_at(road,x)
    for lane,(upper,lower) in enumerate(zip(b,b[1:])):
        if lower+margin<=lateral<=upper-margin:return lane
    width=pocket_width(road.get('left_pockets',[]),x)
    if width>=1.8 and b[0]+margin<lateral<=b[0]+width-margin:return -1
    return None
