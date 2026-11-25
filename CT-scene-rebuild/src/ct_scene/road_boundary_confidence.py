"""Paint support changes regularization strength, never road/actor geometry."""
from .road_geometry import section_at,boundaries_at


def boundary_support(road,station):
    bounds=boundaries_at(road,station);section=section_at(road,station)
    evidence=(section or {}).get('evidence',{})
    peaks=evidence.get('peaks',[])
    # Profiles may contain local evidence; a distant width hypothesis cannot
    # count as repeated paint at the current station.
    if not peaks:
        local=(section or {}).get('observed_width_states',[])
        if local:
            near=min(local,key=lambda r:abs(r['station_m']-station))
            if abs(near['station_m']-station)<=35:peaks=near.get('evidence',{}).get('peaks',[])
    result=[]
    for value in bounds:
        candidates=[p for p in peaks if abs(float(p.get('y',1e6))-value)<=.75]
        best=dict(confidence=.2,kind='unverified_geometry_prior',paint_frames=0)
        for peak in candidates:
            kind=peak.get('kind','unknown');frames=int(peak.get('frames',peak.get('solid_frames',0)))
            if kind in ('white','yellow','yellow_divider','solid_white','dashed_white'):
                confidence=min(1.,frames/12.)
            elif kind=='continuous_right_edge_hypothesis':confidence=min(.6,frames/20.)
            else:confidence=.15
            if confidence>best['confidence']:best=dict(confidence=confidence,kind=kind,paint_frames=frames)
        result.append(dict(lateral_m=float(value),**best))
    return result


def lane_confidence(road,station,lane):
    support=boundary_support(road,station)
    if not isinstance(lane,int) or not 0<=lane<len(support)-1:return .2
    return min(support[lane]['confidence'],support[lane+1]['confidence'])


def membership_supported(actor,row):
    if str(row.get('lane_prior','')).startswith('disabled'):return False
    if actor.get('lane_motion_evidence',{}).get('clear_observations')==0:
        return bool(row.get('lane_measurement_usable') or row.get('paint_lane_supported'))
    return True


def review(road):
    return dict(role='soft_regularization_confidence_only; no_geometry_changes; no_map_lane_truth',sections=[
        dict(id=s['id'],station_m=(s['station_range_m'][0]+s['station_range_m'][1])/2,
             boundaries=boundary_support(road,(s['station_range_m'][0]+s['station_range_m'][1])/2))
        for s in road.get('lane_sections',[])])
