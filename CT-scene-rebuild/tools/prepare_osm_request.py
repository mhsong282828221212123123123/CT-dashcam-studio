"""Prepare a small source-GPS map request; does not transmit or register poses."""
import argparse,json
from pathlib import Path
import av
from ct_scene.telemetry import nal_units,sei_messages

p=argparse.ArgumentParser();p.add_argument('--front',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
points=[]
with av.open(str(a.front)) as c:
    stream=c.streams.video[0];extra=stream.codec_context.extradata;length=(extra[4]&3)+1
    for packet in c.demux(stream):
        try:messages=[m for n in nal_units(bytes(packet),length) for m in sei_messages(n)]
        except ValueError:continue
        for message in messages:
            f=message.get('fields',{});lat=f.get('latitude_deg',{});lon=f.get('longitude_deg',{})
            if lat.get('valid') and lon.get('valid') and -90<float(lat['value'])<90 and -180<float(lon['value'])<180:points.append((float(lat['value']),float(lon['value'])))
if not points:raise ValueError('No source GPS')
lat,lon=zip(*points);bbox=[min(lon)-.003,min(lat)-.003,max(lon)+.003,max(lat)+.003]
if (bbox[2]-bbox[0])*(bbox[3]-bbox[1])>.02:raise ValueError('Bounding box exceeds local cache policy')
record=dict(source=a.front.stem.removesuffix('-front'),gps_points=len(points),bbox=bbox,url='https://api.openstreetmap.org/api/0.6/map?bbox='+','.join(f'{x:.7f}' for x in bbox),role='topology reference only; no lane or pose registration')
a.output.parent.mkdir(parents=True,exist_ok=True)
if a.output.exists():raise FileExistsError(a.output)
a.output.write_text(json.dumps(record,indent=2),encoding='utf-8');print('Source GPS request prepared; no network call')
