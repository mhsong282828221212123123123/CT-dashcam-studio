import unittest
from types import SimpleNamespace as S
import numpy as np
from core.scene_bundle_refinement import reject_unestablished_outliers

class BundleAbstentionTests(unittest.TestCase):
    def test_bundle_does_not_reintroduce_objects_beyond_observed_central_boundary(self):
        from core.scene_bundle_refinement import refine_bundles
        from unittest.mock import patch
        scenes=[S(time=i*.1,actors=[],telemetry=None) for i in range(10)]
        obj=dict(kind='car',track_id='front:1',bbox=[100,500,80,70],track_status='confirmed',road_relation='beyond_central_boundary')
        rows=[dict(cameras={'front':dict(available=True,objects=[obj])}) for _ in scenes]
        with patch('core.scene_motion_bundle.fit_motion_bundle',side_effect=AssertionError('Excluded object reached optimizer')):
            report=refine_bundles(scenes,rows,[(np.zeros(2),np.eye(2))]*10,[{'front':None}]*10,[{} for _ in scenes])
        self.assertEqual(report,[])

    def test_only_brief_unfitted_outliers_abstain_and_pixels_survive(self):
        scenes=[];rows=[];audits=[];poses=[]
        for i in range(10):
            actors=[S(key=k,sources=(s,),x=4*i,z=20.) for k,s in
                    [('short','front:1'),('established','front:2'),('bundle:front:3','front:3')]]
            scenes.append(S(time=i*.1,actors=actors));audits.append({});poses.append((np.zeros(2),np.eye(2)))
            rows.append({'cameras':{'front':{'objects':[dict(track_id=s,track_status='confirmed')
                for s in (['front:1','front:2','front:3'] if i<3 else ['front:2','front:3'])]}}})
        report=reject_unestablished_outliers(scenes,rows,poses,audits)
        self.assertEqual([r['id'] for r in report],['short'])
        self.assertEqual([a.key for a in scenes[0].actors],['established','bundle:front:3'])
        self.assertEqual(len(rows[0]['cameras']['front']['objects']),3)
        self.assertIn('abstained_3d_tracks',audits[0])
