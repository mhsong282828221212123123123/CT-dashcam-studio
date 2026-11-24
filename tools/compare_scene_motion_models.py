"""Read-only proposal comparison using frozen camera horizons and detections."""
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import cameras_for,default_profile
from core.scene_preview import EgoTimeline
from core.scene_world_tracks import WorldTracks,rotation
from core.scene_trajectory_fit import fit_trajectory

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('result',type=Path)
    parser.add_argument('--sources',nargs='+',required=True)
    parser.add_argument('--start',type=float,required=True)
    parser.add_argument('--end',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=[json.loads(s) for s in (args.result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    audit=json.loads((args.result/'fusion-audit.json').read_text(encoding='utf-8'))
    if len(rows)!=len(audit):raise ValueError('Audit/frame mismatch')
    cameras=cameras_for(json.loads((args.result/'manifest.json').read_text(encoding='utf-8')),default_profile())
    world=WorldTracks();tracks={key:[] for key in args.sources}
    for row,a,(data,t,_) in zip(rows,audit,EgoTimeline(rows,.15).states):
        if abs(t-a['time_s'])>1e-6:raise ValueError('Audit timestamp mismatch')
        world.update([],t,data);R=rotation(world.angle);origin=world.pose.copy()
        if not args.start<=t<=args.end:continue
        for name,frame in row['cameras'].items():
            if name not in cameras:continue
            camera=cameras[name]
            if name=='front':camera=replace(camera,horizon=a['front_horizon_px'])
            for obj in frame.get('objects',[]):
                key=obj['track_id']
                if key not in tracks or obj.get('track_status')!='confirmed':continue
                x,y,w,h=obj['bbox']
                local=next((np.array(v['position']) for v in a['world_tracks'] if key in v['sources']),None)
                if local is None:local=camera.ground(x+w/2,y+h)
                seed=origin+R@local if local is not None else np.full(2,np.nan)
                tracks[key].append((dict(time=t,camera=camera,bbox=obj['bbox'],ego_origin=origin,ego_rotation=R),seed))
    results=[]
    for key,items in tracks.items():
        for turn in (False,True):
            proposal=fit_trajectory([v[0] for v in items],[v[1] for v in items],turn_rate=turn)
            results.append(dict(source=key,applied=False,proposal=proposal))
            print(key,turn,{k:v for k,v in (proposal or {}).items() if k!='positions'},flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__':main()
