"""Inspect playable delivery format and MP4 indexing without rewriting media."""
import argparse,json,struct
from pathlib import Path
import av
from ct_scene.ingest import file_hash


def inspect(path,expected_frames=None):
    path=Path(path);atoms=[];size=path.stat().st_size
    with path.open('rb') as f:
        while f.tell()+8<=size:
            offset=f.tell();header=f.read(8);length,kind=struct.unpack('>I4s',header)
            if length==1:
                if offset+16>size:raise ValueError('Truncated extended MP4 atom')
                length=struct.unpack('>Q',f.read(8))[0]
            elif length==0:length=size-offset
            if length<8 or offset+length>size:raise ValueError('Invalid MP4 atom length')
            atoms.append((kind.decode('ascii'),offset));f.seek(offset+length)
    with av.open(str(path)) as c:
        s=c.streams.video[0];r=dict(codec=s.codec_context.name,pixel_format=s.codec_context.format.name,width=s.width,height=s.height,fps=float(s.average_rate),container_frames=s.frames,duration_s=float(c.duration)/av.time_base)
    index={k:v for k,v in atoms};r['faststart']='moov' in index and 'mdat' in index and index['moov']<index['mdat']
    passed=r['codec']=='h264' and r['pixel_format']=='yuv420p' and r['width']==1280 and r['height']==1000 and abs(r['fps']-20)<.01 and r['faststart']
    if expected_frames is not None:passed=passed and r['container_frames']==expected_frames
    return dict(path=str(path.resolve()),bytes=size,sha256=file_hash(path),format_gate='PASS' if passed else 'FAIL',**r)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('video',type=Path);p.add_argument('--frames',type=int);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    r=inspect(a.video,a.frames);a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r));raise SystemExit(0 if r['format_gate']=='PASS' else 2)
