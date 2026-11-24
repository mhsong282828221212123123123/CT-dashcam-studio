"""Reanalyse road evidence without rerunning vehicles or modifying observations."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_road_model import RoadEvidenceDetector
from core.scene_analysis import CameraReader


def enrich(result,source,output,detector=None):
    result,output=Path(result),Path(output)
    if output.exists():raise FileExistsError(output)
    detector=detector or RoadEvidenceDetector('../scene-models/yolop/yolop-640-640.onnx')
    rows=[json.loads(s) for s in (result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((result/'manifest.json').read_text(encoding='utf-8'))
    reader=CameraReader(source)
    try:
        for row in rows:
            _,frame=reader.read_at(row['time_s'])
            camera=row['cameras']['front']
            camera['pitch_lanes']=camera.get('lanes',[])
            if frame is None:
                camera['road_evidence']={'cross_sections':[]};camera['lanes']=[]
                continue
            evidence=detector.detect(frame,camera.get('objects',[]))
            camera['lanes']=evidence['lanes'];camera['road_evidence']=evidence
    finally:reader.close()
    output.mkdir(parents=True)
    manifest['road_detector']=detector.info
    manifest['lanes']='camera segmentation curves; unknown boundary semantics'
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'observations.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf-8')
    print('Road-enriched',len(rows),'frames',output,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('result','source','output'):p.add_argument('--'+name,required=True)
    a=p.parse_args();enrich(a.result,a.source,a.output)
