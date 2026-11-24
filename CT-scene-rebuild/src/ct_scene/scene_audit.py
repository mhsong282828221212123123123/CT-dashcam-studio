"""Observation-to-world coverage, explicitly not a detection accuracy score."""
from collections import defaultdict
import numpy as np


def visible_ids_at(presence,t,tolerance=.12):
    visible=set()
    for key,times in presence.items():
        index=int(np.searchsorted(times,t));options=[i for i in (index-1,index) if 0<=i<len(times)]
        if options and min(abs(times[i]-t) for i in options)<=tolerance:visible.add(key)
    return visible


def observation_coverage(observations,actors,render_states,excluded,original_ids,rear_review=None):
    membership={key:a['global_id'] for a in actors for key in a['local_ids']}
    aliases={key:root for root,ids in original_ids.items() for key in ids}
    reasons={e['id']:e['reason'] for e in excluded};times=np.array([s['time_s'] for s in render_states]);counters=defaultdict(lambda:defaultdict(int));missing=[];ghost_conflicts=[]
    for row in observations:
        if row['support'] not in ('detector_observation','flow_observation'):continue
        camera=row['camera_id'];counter=counters[camera];counter['observed_samples']+=1
        key=row['local_track_id'];gid=membership.get(key);reason=None
        index=int(np.searchsorted(times,row['camera_time_s']));options=[i for i in (index-1,index) if 0<=i<len(times)]
        index=min(options,key=lambda i:abs(times[i]-row['camera_time_s']))
        states={a['id']:a for a in render_states[index]['actors']}
        if camera=='back':
            counter['reference_only_samples']+=1
            refs=[i for i in (rear_review or {}).get('intervals',[]) if i['rear_id']==key and i['start_s']<=row['camera_time_s']<=i['end_s']]
            gids={i['global_id'] for i in refs}
            if len(gids)==1:
                gid=next(iter(gids));counter['identity_reference_assigned']+=1
                if gid in states and abs(times[index]-row['camera_time_s'])<=.1:
                    counter['identity_reference_with_world_state']+=1
                    if states[gid]['ghost']:
                        counter['observed_but_world_ghost']+=1
                        ghost_conflicts.append({'frame_id':row['frame_id'],'local_id':key,'global_id':gid,'time_s':row['camera_time_s']})
                else:counter['identity_reference_geometry_unresolved']+=1
            else:counter['identity_reference_unresolved']+=1
            continue
        if gid is None:reason=reasons.get(aliases.get(key,key),'outside_vehicle_prototype_or_unresolved')
        elif abs(times[index]-row['camera_time_s'])>.1 or gid not in states:reason='world_time_support_missing'
        else:
            counter['covered_in_world_state']+=1
            if states[gid]['ghost']:
                counter['observed_but_world_ghost']+=1
                ghost_conflicts.append({'frame_id':row['frame_id'],'local_id':key,'global_id':gid,'time_s':row['camera_time_s']})
        if reason:
            counter['not_in_world_state']+=1;counter[reason]+=1
            missing.append({'frame_id':row['frame_id'],'camera_id':camera,'local_id':key,'global_id':gid,'time_s':row['camera_time_s'],'reason':reason})
    return {'scope':'Tracked observation coverage, including false/background/out-of-scope detections; not GT accuracy',
        'camera_summary':{c:dict(v) for c,v in counters.items()},'missing_or_excluded':missing,'observed_ghost_conflicts':ghost_conflicts,
        'screen_frustum_visibility':'not_measured_by_this_audit','rear_geometry':'disabled'}
