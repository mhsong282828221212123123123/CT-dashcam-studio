"""Raw SEI and authorized small-area OSM caches for six new-date controls."""
import json
from pathlib import Path
from ct_scene.telemetry import audit_sei
from ct_scene.osm_context import fetch

out=Path('runs/new-holdout-context-001');out.mkdir(exist_ok=True);results=[]
for selection in ['runs/new-date-selection-003/selection.json','runs/new-date-selection-005/selection.json']:
    for source in json.loads(Path(selection).read_text())['selected']:
        sid=source['source_id'];dest=out/sid
        if not dest.exists():audit_sei(source['source_files']['front'],dest,source['last_s'])
        try:r=fetch(dest,Path('map-cache')/(sid+'.osm'))
        except Exception as e:r=dict(status='failed',error=str(e))
        results.append(dict(source_id=sid,**r));(out/'fetch.json').write_text(json.dumps(results,indent=2))
        print(results[-1],flush=True)
