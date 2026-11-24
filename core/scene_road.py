"""Road display from camera boundaries, independent of OSM lane counts."""
import numpy as np


def _lines(segments):
    lines=[]
    for segment in segments:
        points=np.asarray(segment.get('points',[]),float)
        if points.ndim!=2 or points.shape[0]<2 or points.shape[1]!=2 or not np.isfinite(points).all():continue
        if np.ptp(points[:,1])<1.:continue
        slope,intercept=np.polyfit(points[:,1],points[:,0],1)
        if abs(slope)>.6 or abs(intercept)>15:continue
        lines.append((float(slope),float(intercept)))
    lines.sort(key=lambda line:line[1]+8*line[0])
    distinct=[]
    for line in lines:
        if not distinct or abs(line[1]+8*line[0]-distinct[-1][1]-8*distinct[-1][0])>1.2:distinct.append(line)
    return distinct


def add_travel_corridors(scenes,poses,audits,width=3.5):
    from core.scene_vehicle_road import add_vehicle_corridors
    add_vehicle_corridors(scenes,poses,audits,width)


def add_camera_corridors(scenes,poses,audits,width=3.5):
    # Only the camera supplies boundary count/spacing. A host's future path and
    # nearby vehicles cannot create lanes. Width argument is legacy, unused.
    history=None
    road_history=[]
    lane_history=[]
    for i,(scene,audit) in enumerate(zip(scenes,audits)):
        scene.observed_lane_segments=list(scene.lanes)
        scene.road_surface=[];scene.road_surfaces=[];scene.road_view=None
        if hasattr(scene,'camera_road_sections'):
            origin,R=poses[i]
            if any('tracks reset' in event.get('reason','') for event in audit.get('world_track_events',[])):
                road_history=[]
                lane_history=[]
            sections=scene.camera_road_sections
            surfaces=[]
            for first,second in zip(sections,sections[1:]):
                a,b=np.asarray(first),np.asarray(second)
                if np.max(np.linalg.norm(a-b,axis=1))>12:continue
                surfaces.append(np.array([a[0],a[1],b[1],b[0]]))
            # Preserve measured curvature and independent road extent. Old frames
            # supply road behind the host; do not fabricate an ego-centred grid.
            road_history=[item for item in road_history if 0<=scene.time-item[0]<=2.]
            if surfaces and (not road_history or scene.time-road_history[-1][0]>=.2):
                road_history.append((scene.time,[surface@R.T+origin for surface in surfaces]))
            scene.road_surfaces=[((surface-origin)@R).tolist()
                                 for _,items in road_history for surface in items]
            for lane in scene.lanes:
                lane['dashed']=True
                lane['edge_role']='unclassified_camera_paint'
            scene.lanes.extend({'points':points,'dashed':False,'observed':False,
                                'inferred':True,'source':'segmented_road_extent',
                                'edge_role':'estimated_road_edge','extrapolated':False}
                               for points in getattr(scene,'camera_road_edges',[]))
            current_lanes=list(scene.lanes)
            lane_history=[item for item in lane_history if 0<=scene.time-item[0]<=2.]
            # Recover paint already driven over in world coordinates. Each past
            # frame contributes only the newly exposed longitudinal band, so
            # overlapping old observations do not draw multiple parallel lines.
            covered_z=min((p[1] for lane in current_lanes for p in lane['points']),default=3.)
            for _,past in reversed(lane_history):
                local=[(lane,(points-origin)@R) for lane,points in past]
                near=min((points[:,1].min() for _,points in local if len(points)),default=covered_z)
                if near>=covered_z:continue
                for lane,points in local:
                    selected=points[(points[:,1]<covered_z)&(points[:,1]>=max(-15.,near))]
                    if len(selected)<2:continue
                    scene.lanes.append(dict(lane,points=selected.tolist(),observed=False,inferred=True,
                                            source='past_camera_paint_world_coordinates'))
                covered_z=near
                if covered_z<=-15:break
            if current_lanes and (not lane_history or scene.time-lane_history[-1][0]>=.2):
                lane_history.append((scene.time,[(dict(lane),np.asarray(lane['points'])@R.T+origin)
                                                for lane in current_lanes]))
            audit['travel_corridors']={'source':'camera_road_segmentation',
                'road_quads':len(scene.road_surfaces),'outer_edges_confirmed':False,
                'lane_count_inferred':False,'temporal_road_memory_s':2.}
            history=None
            continue
        lines=_lines(scene.observed_lane_segments);held=False
        if len(lines)>=2:
            slopes=np.array([line[0] for line in lines]);offsets=np.array([line[1] for line in lines])
            gaps=np.diff(offsets)
            # Crossing branches/intersections must not become a straight lane grid.
            valid=np.ptp(slopes)<.12 and np.all((gaps>2)&(gaps<5.5)) and offsets[-1]-offsets[0]<20
            if not valid:lines=[];history=None
        if len(lines)<2:
            lines=[]
            if history is not None and 0<scene.time-history[0]<=1.:
                origin,R=poses[i]
                segments=[{'points':((p-origin)@R).tolist()} for p in history[1]]
                lines=_lines(segments);held=True
        if len(lines)>=2:
            slope=float(np.median([line[0] for line in lines]))
            offsets=[line[1] for line in lines]
            zs=np.linspace(-12,45,115)
            boundaries=[[(float(slope*z+x),float(z)) for z in zs] for x in offsets]
            scene.lanes=[{'points':points,'dashed':0<j<len(boundaries)-1,
                'observed':False,'inferred':True,'source':'camera_boundary_display',
                'edge_role':'outer_display_boundary' if j in (0,len(boundaries)-1) else 'internal',
                'extrapolated':True,'temporally_held':held} for j,points in enumerate(boundaries)]
            scene.road_surface=boundaries[-1]+boundaries[0][::-1]
            if not held:
                origin,R=poses[i]
                history=(scene.time,[np.asarray([line[0],line[-1]])@R.T+origin for line in boundaries])
            audit['travel_corridors']={'source':'camera_boundaries_extrapolated_for_display',
                'display_lane_count':len(boundaries)-1,'outer_edges_confirmed':False,
                'temporally_held':held,'observed_segments_preserved':len(scene.observed_lane_segments)}
        else:
            audit['travel_corridors']={'source':'image_segments_only_insufficient_road_support',
                'observed_segments_preserved':len(scene.lanes),'synthetic_lane_count':0}
    # Offline reconstruction can use later camera views of the same road.
    # Keep current nearby evidence; future paint only fills the farther bands.
    for i,scene in enumerate(scenes):
        if not hasattr(scene,'camera_road_sections'):continue
        origin,R=poses[i]
        far=max((p[1] for lane in scene.observed_lane_segments for p in lane['points']),default=5.)
        last_time=scene.time
        for j in range(i+1,len(scenes)):
            future=scenes[j]
            if future.time-scene.time>2.:break
            if future.time-scenes[j-1].time>.3:break
            if any('tracks reset' in e.get('reason','') for e in audits[j].get('world_track_events',[])):break
            if not hasattr(future,'camera_road_sections'):break
            if future.time-last_time<.2:continue
            last_time=future.time
            next_origin,next_R=poses[j]
            transform=lambda points:((np.asarray(points)@next_R.T+next_origin)-origin)@R
            for a,b in zip(future.camera_road_sections,future.camera_road_sections[1:]):
                if np.max(np.linalg.norm(np.asarray(a)-b,axis=1))>12:continue
                quad=transform([a[0],a[1],b[1],b[0]])
                if quad[:,1].max()>0 and quad[:,1].min()<65:
                    scene.road_surfaces.append(quad.tolist())
            raw=[dict(lane) for lane in future.observed_lane_segments]
            raw.extend({'points':p,'dashed':False,'edge_role':'estimated_road_edge'}
                       for p in getattr(future,'camera_road_edges',[]))
            next_far=far
            for lane in raw:
                points=transform(lane['points'])
                if len(points):next_far=max(next_far,float(points[:,1].max()))
                selected=points[(points[:,1]>far)&(points[:,1]<65)]
                if len(selected)<2:continue
                scene.lanes.append(dict(lane,points=selected.tolist(),observed=False,inferred=True,
                                        dashed=lane.get('edge_role')!='estimated_road_edge',
                                        source='future_camera_paint_world_coordinates'))
            far=next_far
        audits[i]['travel_corridors']['future_camera_window_s']=2.
        audits[i]['travel_corridors']['road_quads']=len(scene.road_surfaces)
    from core.scene_road_fit import coherent_paths,retain_unmatched_observations
    for i,scene in enumerate(scenes):
        if not hasattr(scene,'camera_road_sections'):continue
        origin,R=poses[i];segments=[];surface_frames=[]
        for direction in (-1,1):
            index=i if direction==1 else i-1
            last_time=None
            while 0<=index<len(scenes) and abs(scenes[index].time-scene.time)<=2.:
                candidate=scenes[index]
                between=index if direction==1 else index+1
                previous_index=index-direction
                if index!=i and 0<=previous_index<len(scenes) and abs(candidate.time-scenes[previous_index].time)>.3:break
                if index!=i and any('tracks reset' in e.get('reason','') for e in audits[between].get('world_track_events',[])):break
                if not hasattr(candidate,'camera_road_sections'):break
                if last_time is None or abs(candidate.time-last_time)>=.2:
                    last_time=candidate.time
                    o,Q=poses[index]
                    quads=[]
                    for first,second in zip(candidate.camera_road_sections,candidate.camera_road_sections[1:]):
                        a,b=np.asarray(first),np.asarray(second)
                        if np.max(np.linalg.norm(a-b,axis=1))>12:continue
                        quad=np.array([a[0],a[1],b[1],b[0]])
                        quads.append(((quad@Q.T+o-origin)@R).tolist())
                    if quads:surface_frames.append(quads)
                    raw=[dict(lane) for lane in candidate.observed_lane_segments]
                    raw.extend({'points':p,'edge_role':'estimated_road_edge'} for p in getattr(candidate,'camera_road_edges',[]))
                    for lane in raw:
                        points=((np.asarray(lane['points'])@Q.T+o)-origin)@R
                        segments.append(dict(lane,points=points.tolist()))
                index+=direction
        from core.scene_road_surface import consensus_surface
        if surface_frames:
            scene.road_surfaces=consensus_surface(surface_frames)
        audits[i]['travel_corridors']['road_surface_method']='temporal_cross_section_consensus'
        audits[i]['travel_corridors']['road_quads']=len(scene.road_surfaces)
        fitted=coherent_paths(segments)
        # A fit is only used where multiple observations support a path. Keep
        # the measured fragments if no coherent camera geometry can be found.
        supported=[lane for lane in fitted if lane['observation_segments']>=3]
        if supported:
            unmatched=retain_unmatched_observations(scene.observed_lane_segments,supported)
            scene.lanes=supported+unmatched
            audits[i]['travel_corridors']['coherent_camera_paths']=len(supported)
            audits[i]['travel_corridors']['unmatched_current_camera_paths']=len(unmatched)
