"""Fail closed unless independent, complete sequence reviews meet every gate.

Detector coverage and reprojection reports are deliberately not accepted as
manual sequence reviews. No averaging may hide a failed approaching vehicle.
"""
import argparse,json,math
from pathlib import Path

REQUIRED={'day','night','stopped','low_speed','high_speed','curve','intersection',
          'lane_change','side_approach','occlusion','truck','motorcycle','pedestrian','parking'}


def evaluate(reviews):
    failures=[];coverage={key:set() for key in REQUIRED}
    for review in reviews:
        name=review.get('id','unnamed')
        if review.get('split')!='held_out':continue
        if review.get('independent_manual_annotation') is not True or review.get('full_sequence_reviewed') is not True:
            failures.append(f'{name}: incomplete independent review');continue
        duration=review.get('duration_s',0)
        if not isinstance(duration,(int,float)) or not math.isfinite(duration) or duration<30:
            failures.append(f'{name}: less than 30 seconds');continue
        if not review.get('event_id'):
            failures.append(f'{name}: missing event identity');continue
        for scenario in review.get('verified_scenarios',[]):
            if scenario in coverage:coverage[scenario].add(review['event_id'])
        metrics=review.get('metrics',{})
        checks={'critical_vehicle_display_recall':lambda x:.95<=x<=1.,
                'longest_visible_vehicle_dropout_s':lambda x:x<=.5,
                'wrong_lane_events':lambda x:x==0,
                'duplicate_vehicle_events':lambda x:x==0,
                'wrong_lane_count_events':lambda x:x==0,
                'ego_pinned_road_events':lambda x:x==0,
                'invented_intersection_lane_events':lambda x:x==0,
                'lane_reprojection_error_p95_image_width':lambda x:x<=.01,
                'unresolved_critical_events':lambda x:x==0}
        for key,predicate in checks.items():
            value=metrics.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<0 or not predicate(value):
                failures.append(f'{name}: missing or failing {key}')
    for key,events in coverage.items():
        if len(events)<3:failures.append(f'{key}: only {len(events)} independently reviewed events; need 3')
    return dict(status='PASS' if not failures else 'FAIL',failures=failures,
                reviewed_events_by_scenario={key:len(events) for key,events in coverage.items()},
                scope='Project visual acceptance only; not metric accuracy certification')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--reviews',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();path=Path(args.reviews)
    reviews=json.loads(path.read_text(encoding='utf-8')) if path.exists() else []
    result=evaluate(reviews)
    Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(result['status'],len(result['failures']),'unmet gates')
    raise SystemExit(0 if result['status']=='PASS' else 1)
