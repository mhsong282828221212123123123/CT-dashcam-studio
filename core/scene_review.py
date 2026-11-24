"""Evidence-based review of sparse, manually checked observations.

Reference labels are not detector outputs and uncertain regions never enter recall.
This reports selected-target agreement, not full precision/recall or metric 3D accuracy.
"""
import numpy as np
from core.scene_analysis import iou,optimal_pairs,vehicle_family


def compare_targets(targets,detections,threshold=.3):
    confirmed=[t for t in targets if t.get('status')=='confirmed']
    scores=np.zeros((len(confirmed),len(detections)))
    for i,target in enumerate(confirmed):
        for j,detection in enumerate(detections):
            score=iou(target['bbox'],detection['bbox'])
            if vehicle_family(target['kind'])==vehicle_family(detection['kind']) and score>=threshold:
                scores[i,j]=score
    pairs=optimal_pairs(scores)
    matched={i:j for i,j in pairs}
    results=[]
    for i,target in enumerate(confirmed):
        j=matched.get(i)
        results.append({'target_id':target['id'],'status':'matched' if j is not None else 'missed',
            'detection_index':j,'iou':float(scores[i,j]) if j is not None else None,
            'detected_kind':detections[j]['kind'] if j is not None else None,
            'reference_subtype':target.get('subtype')})
    return {'targets':len(confirmed),'matched':len(pairs),'missed':len(confirmed)-len(pairs),
        'uncertain_targets':sum(t.get('status')!='confirmed' for t in targets),
        'unmatched_detections':[j for j in range(len(detections)) if j not in matched.values()],
        'results':results}


def check_relations(relations,target_to_actor):
    outcomes=[]
    for relation in relations:
        a=target_to_actor.get(relation['a'])
        b=target_to_actor.get(relation['b']) if relation['b']!='ego' else {'x':0.,'z':0.}
        outcome='unassessable'
        if a is not None and b is not None:
            if relation['type']=='left_of': passed=a['x']>b['x']+.5
            elif relation['type']=='right_of': passed=a['x']<b['x']-.5
            elif relation['type']=='ahead_of': passed=a['z']>b['z']+1.
            elif relation['type']=='centered': passed=abs(a['x']-b['x'])<.8
            else: raise ValueError('Unsupported relation')
            outcome='pass' if passed else 'fail'
        outcomes.append(dict(relation,outcome=outcome))
    return outcomes
