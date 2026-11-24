"""Two authorized small GPS-region requests; immutable cached OSM outlines."""
import json
from pathlib import Path
from ct_scene.osm_context import fetch,run

out=Path('runs/new-date-osm-fetch-002');out.mkdir(exist_ok=False);results=[]
for sid in ['2026-08-17_12-37-37','2026-10-26_12-01-20']:
    telemetry=Path('runs/new-date-suite-002')/sid/'telemetry';cache=Path('map-cache')/(sid+'.osm')
    try:
        result=fetch(telemetry,cache);dest=out/sid;run(telemetry,cache,dest)
        result['report']=json.loads((dest/'report.json').read_text())
    except Exception as e:result=dict(status='failed',error=str(e))
    results.append(dict(source_id=sid,**result));print(json.dumps(results[-1]),flush=True)
    (out/'fetch.json').write_text(json.dumps(results,indent=2))
