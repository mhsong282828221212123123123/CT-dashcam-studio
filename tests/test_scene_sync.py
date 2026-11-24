import unittest
import tempfile
from pathlib import Path
from fractions import Fraction
import numpy as np

from core.scene_sync import SequenceAlignment


def cache(sequences):
    return {i: dict(frame_seq_no=s, speed_mps=s/100, heading=s/10,
                    steering_deg=0, brake_applied=False) for i,s in enumerate(sequences)}


class SequenceSyncTests(unittest.TestCase):
    def test_shared_counter_handles_offset_and_dropped_frames(self):
        a=cache(range(100,120));b=cache([s for s in range(101,120) if s!=110])
        sync=SequenceAlignment(a,b,len(a),len(b))
        self.assertEqual(sync.report['status'],'sequence_matched')
        self.assertEqual(sync.target_index(1),0)
        self.assertIsNone(sync.target_index(0))
        self.assertIsNone(sync.target_index(10))
        self.assertEqual(sync.target_index(11),9)

    def test_unrelated_rear_counter_is_rejected(self):
        a=cache(range(100,120));b=cache(range(900,920))
        sync=SequenceAlignment(a,b,20,20)
        self.assertEqual(sync.report['status'],'unverified')
        self.assertIsNone(sync.target_index(5))

    def test_repeated_sequence_and_partial_metadata_are_rejected(self):
        a=cache(range(20))
        for b in [cache([0]*20),cache(range(19)),{**a,2:{**a[2],'frame_seq_no':None}}]:
            self.assertEqual(SequenceAlignment(a,b,20,20).report['status'],'unverified')

    def test_shared_counter_with_incompatible_vehicle_state_is_rejected(self):
        a=cache(range(20));b=cache(range(20))
        for row in b.values():row['speed_mps']+=10
        self.assertEqual(SequenceAlignment(a,b,20,20).report['status'],'unverified')

    def test_heading_wrap_is_compatible_but_not_reported_as_exposure_survey(self):
        a=cache(range(20));b=cache(range(20))
        for row in a.values():row['heading']=359.9
        for row in b.values():row['heading']=.1
        sync=SequenceAlignment(a,b,20,20)
        self.assertEqual(sync.report['status'],'sequence_matched')
        self.assertIn('not independently measured',sync.report['limitation'])


class PresentationSamplingTests(unittest.TestCase):
    def test_missing_optional_pts_reader_is_explicitly_unverified(self):
        from unittest.mock import patch
        from core.scene_sync import presentation_times
        with patch.dict('sys.modules',{'av':None}):
            times,report=presentation_times('missing.mp4',20)
        self.assertIsNone(times)
        self.assertEqual(report['status'],'nominal_fps_unverified')

    def test_variable_rate_sampling_uses_pts_and_reads_corresponding_real_pixels(self):
        import av
        from core.scene_analysis import CameraReader
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'variable.mp4'
            with av.open(str(path),'w') as container:
                stream=container.add_stream('mpeg4',rate=25)
                stream.width=64;stream.height=48;stream.pix_fmt='yuv420p'
                stream.codec_context.time_base=Fraction(1,1000)
                for i,pts in enumerate([0,40,80,200,240,500]):
                    frame=av.VideoFrame.from_ndarray(np.full((48,64,3),i*30,np.uint8),format='bgr24')
                    frame.pts=pts;frame.time_base=Fraction(1,1000)
                    for packet in stream.encode(frame):container.mux(packet)
                for packet in stream.encode():container.mux(packet)
            reader=CameraReader(path)
            try:
                self.assertEqual(reader.timing['status'],'container_pts')
                self.assertEqual(reader.index_at(.15),3)
                index,image=reader.read_at(.15)
                self.assertEqual(index,3)
                self.assertAlmostEqual(float(image.mean()),90,delta=4)
                self.assertAlmostEqual(reader.timestamps[index],.2,places=5)
                self.assertIsNone(reader.read_at(1.)[1])
            finally:reader.close()
