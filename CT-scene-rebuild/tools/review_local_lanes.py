"""Regression audit for the October 21 white-car lane-prior correction.

Fixture IDs below are validation references, never production lane rules.
This checks invariants and the reported defect, not general metric accuracy.
"""
import json
from pathlib import Path
import av
import numpy as np
from PIL import Image, ImageDraw


out = Path('runs/local-lane-review-001')
out.mkdir(exist_ok=True)
results = {}
for name, old_name, new_name, stamps in [
    ('oct21', 'wp06-wp07-oct21-004', 'oct21-local-lanes-003', [33, 37, 38, 39, 40, 41]),
    ('sep17', 'wp06-wp07-sep17-004', 'sep17-local-lanes-002', [0, 13, 19, 30, 45, 58]),
]:
    old, new = (Path('runs') / n for n in (old_name, new_name))
    load = lambda p: json.loads(p.read_text(encoding='utf-8'))
    before, after = (load(p / 'report.json') for p in (old, new))
    actors_old, actors_new = (load(p / 'actors.json') for p in (old, new))
    by_old = {a['id']: a for a in actors_old}
    checks = {
        'road_unchanged': before['road'] == after['road'],
        'ego_lane_state_exact': (old/'ego-lane-state.json').read_bytes() == (new/'ego-lane-state.json').read_bytes(),
        'identity_edges_exact': load(old/'association-review.json') == load(new/'association-review.json'),
        'actor_ids_unchanged': set(by_old) == {a['id'] for a in actors_new},
        'frozen_lead_exact': all(a['samples'] == by_old[a['id']]['samples'] for a in actors_new if a['frozen_lead']),
        'longitudinal_positions_unchanged': all(np.allclose([r['position_W'][0] for r in a['samples']], [r['position_W'][0] for r in by_old[a['id']]['samples']], atol=1e-9, rtol=0) for a in actors_new),
    }
    states = [json.loads(l) for l in (new/'render-states.jsonl').read_text().splitlines()]
    checks['finite_render_states'] = all(np.isfinite(a['position_W'] + [a['yaw_rad']]).all() for s in states for a in s['actors'])
    checks['no_source_visible_ghosts'] = all(not (a['source_visible'] and a['ghost']) for s in states for a in s['actors'])
    if name == 'oct21':
        white = next(a for a in actors_new if a['id'] == 'right_repeater-0010')
        black = next(a for a in actors_new if a['id'] == 'right_repeater-0009')
        checks['white_stable_lane_without_approach_change'] = all(r['lane_index'] == 1 for r in white['samples'])
        nearest = lambda a: min(a['samples'], key=lambda r: abs(r['time_s']-39))
        checks['white_black_separate_lanes_at39'] = nearest(white)['lane_index'] != nearest(black)['lane_index']
        checks['lateral_separation_at39_over2m'] = abs(nearest(white)['position_W'][1]-nearest(black)['position_W'][1]) > 2
    sheet = Image.new('RGB', (1920, 1000))
    frame_count = 0
    with av.open(str(new/'surrounding-world.mp4')) as container:
        wanted = {round(t*20): i for i, t in enumerate(stamps)}
        for frame in container.decode(video=0):
            if frame_count in wanted:
                i = wanted[frame_count]
                sheet.paste(frame.to_image().resize((640, 500)), ((i%3)*640, (i//3)*500))
            frame_count += 1
        codec = container.streams.video[0].codec_context.name
    checks['full_export_decoded'] = frame_count == after['frames']
    checks['phone_h264'] = codec == 'h264'
    sheet.save(out/f'{name}.jpg', quality=94)
    results[name] = dict(output=str(new), frames=frame_count, duration_s=after['duration_s'], checks=checks)
    assert all(checks.values()), results[name]
(out/'review.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(results, ensure_ascii=False, indent=2))
