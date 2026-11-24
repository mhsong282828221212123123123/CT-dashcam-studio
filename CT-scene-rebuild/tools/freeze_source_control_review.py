"""Record human raw-source review and gates before reading scene predictions."""
import json
from pathlib import Path
from datetime import datetime, timezone

root=Path('runs/dense-source-controls-001')
p=root/'objects.json'
data=json.loads(p.read_text())
notes={
 'brown_compact':'All 50 raw crops reviewed: same rear vehicle continuously visible; no occlusion or lane change in interval.',
 'white_taxi':'All 193 raw crops reviewed: same white taxi continuously visible ahead. Cross traffic behind it is distinct; target stays in own lane.',
 'green_ioniq5':'All 201 raw crops reviewed: same green lead continuously visible; right parked SUV is distinct. Lead creeps after about 29 s; do not assert stationary world coordinates.',
 'white_bmw':'All 101 raw crops reviewed: same left adjacent BMW continuously visible with nearly fixed image position. Identity before 30 s is not labelled.',
}
for target in data['targets']:
 target['dense_visibility_reviewed']=True
 target['review_notes']=notes[target['object_id']]
 target['visibility_scope']='5 Hz source samples; continuous visibility inferred between adjacent reviewed samples, not exhaustive full-rate labels'
p.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
gates=dict(created_utc=datetime.now(timezone.utc).isoformat(),
 scope='Selected source-labelled critical vehicles only. Not exhaustive detection recall or metric ground truth.',
 frozen_before_scene_prediction_inspection=True,
 anchor_iou_min=.4,anchor_match_margin_min=.1,anchor_time_tolerance_s=.3,
 solid_coverage_min=.95,longest_absence_max_s=.5,
 expected_lane_fraction_min=.95,expected_lane_count_fraction_min=.95,
 same_direction_fraction_min=.95,max_relative_heading_deg=45.,
 lead_ahead_fraction_min=.95,
 warning='Provisional engineering rejection gates, not commercial quality certification. Lane counts apply only to source-labelled one-lane intervals.')
(root/'gates.json').write_text(json.dumps(gates,ensure_ascii=False,indent=2),encoding='utf-8')
print('Recorded four dense source reviews and frozen independent gates.')
