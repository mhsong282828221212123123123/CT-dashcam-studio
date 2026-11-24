from fractions import Fraction
from pathlib import Path
import tempfile
import json
import unittest
import av
from PIL import Image
from ct_scene.annotation import prepare, validate_annotations


class AnnotationTests(unittest.TestCase):
    def test_exact_grid_no_extra_boundary_and_review_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            source=root/'synthetic-front.mp4'
            with av.open(str(source),'w') as container:
                stream=container.add_stream('mpeg4',rate=5)
                stream.width=64;stream.height=48;stream.pix_fmt='yuv420p'
                for i in range(176):
                    f=av.VideoFrame.from_image(Image.new('RGB',(64,48),(i%255,20,40)))
                    f.pts=i;f.time_base=Fraction(1,5)
                    for packet in stream.encode(f):container.mux(packet)
                for packet in stream.encode():container.mux(packet)
            package=root/'package'
            prepare(source,package,35,5)
            frames=json.loads((package/'frames.json').read_text())
            self.assertEqual(len(frames),175)
            self.assertAlmostEqual(frames[-1]['relative_request_s'],34.8)
            self.assertEqual(frames[0]['space_id'],'raw')
            self.assertIsNone(frames[0]['session_time_s'])
            labels=root/'labels.json'
            label=dict(frame_id=frames[0]['frame_id'],object_id='A',bbox_xyxy=[1,2,30,40],reviewed=True)
            data={'objects':[label], 'reviewed_frames':[]}
            labels.write_text(json.dumps(data))
            with self.assertRaises(ValueError):validate_annotations(package,labels)
            data['reviewed_frames']=[frames[0]['frame_id'],frames[1]['frame_id']]
            labels.write_text(json.dumps(data))
            result=validate_annotations(package,labels)
            self.assertEqual(result['labels'],1)
            self.assertEqual(result['explicitly_reviewed_frames'],2)
            data['objects'].append(label.copy());labels.write_text(json.dumps(data))
            with self.assertRaises(ValueError):validate_annotations(package,labels)
