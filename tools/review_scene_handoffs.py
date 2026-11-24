"""Replay optional handoff experiment; never treats geometric joins as acceptance."""
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline
from core.scene_handoff import stitch_handoffs

root = Path(sys.argv[1])
overrides_path = root / 'split-overrides.json'
overrides = json.loads(overrides_path.read_text(encoding='utf-8')) if overrides_path.exists() else {}
cases = {c['id']: c for c in json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))}
reports = []
for variant in ('baseline', 'continuous'):
    for path in sorted((root/variant).glob('*/manifest.json')):
        manifest = json.loads(path.read_text(encoding='utf-8'))
        rows = [json.loads(x) for x in (path.parent/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        timeline = FusedSceneTimeline(rows, manifest, enable_handoffs=False)
        count = stitch_handoffs(timeline.scenes, rows, timeline.ego_poses, timeline.audit,
                               {c:d['width'] for c,d in manifest['cameras'].items()})
        case = cases[path.parent.name]
        split = overrides.get(case['date'], {}).get('split', case['split'])
        reports.append(dict(id=case['id'],variant=variant,split=split,merged_tracks=count,
            events=[dict(time_s=a['time_s'],handoffs=a['handoffs']) for a in timeline.audit if a.get('handoffs')],
            visual_acceptance=False))
        print(variant, case['id'], count, flush=True)
        (root/'handoff-bidirectional-experiment.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
