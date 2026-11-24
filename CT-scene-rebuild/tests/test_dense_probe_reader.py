from fractions import Fraction
from pathlib import Path
import tempfile
import unittest
import av
from PIL import Image
from ct_scene.ingest import file_hash
from ct_scene.side_transfer_probe import dense_samples


class DenseProbeReaderTests(unittest.TestCase):
    def test_exact_pts_anchor_forward_backward_and_wrong_source(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'synthetic.mp4'
            with av.open(str(source),'w') as container:
                stream=container.add_stream('mpeg4',rate=12);stream.width=64;stream.height=48;stream.pix_fmt='yuv420p'
                for i in range(24):
                    frame=av.VideoFrame.from_image(Image.new('RGB',(64,48),(i*8,20,40)))
                    frame.pts=i;frame.time_base=Fraction(1,12)
                    for packet in stream.encode(frame):container.mux(packet)
                for packet in stream.encode():container.mux(packet)
            with av.open(str(source)) as container:
                f=list(container.decode(video=0))[12];pts=f.pts;t=float(f.pts*f.time_base)
            anchor={'source_sha256':file_hash(source),'pts':pts,'camera_time_s':t,'camera_id':'left_repeater'}
            forward=list(dense_samples(source,anchor,.5));backward=list(dense_samples(source,anchor,.5,-1))
            self.assertEqual(forward[0][0]['pts'],pts);self.assertEqual(backward[0][0]['pts'],pts)
            self.assertGreater(len(forward),5)
            self.assertTrue(all(a[0]['camera_time_s']<b[0]['camera_time_s'] for a,b in zip(forward,forward[1:])))
            self.assertTrue(all(a[0]['camera_time_s']>b[0]['camera_time_s'] for a,b in zip(backward,backward[1:])))
            with self.assertRaises(ValueError):list(dense_samples(source,dict(anchor,source_sha256='wrong'),.5))
