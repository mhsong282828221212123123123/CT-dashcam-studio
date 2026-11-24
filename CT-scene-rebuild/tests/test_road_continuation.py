import unittest
from ct_scene.road_continuation import support_gaps
from ct_scene.road_geometry import support_kind


def fixture():
    line=dict(points_W_xy=[[0,0],[150,0]],unsupported_intervals_m=[[40,100]],supported_extent_m=[0,150])
    road=dict(boundaries_y=[1.75,-1.75],reference_line=line,junctions=[]);segments=[]
    for k,s in enumerate([50,70,90]):
        for j in range(2):
            for d in [1.75,-1.75]:segments.append(dict(kind='white',ground_length_m=8,color_continuity=.95,frame_id=f'{k}-{j}',time_s=k+j*.2,points_W_xy=[[s-4,d],[s+4,d]]))
    return road,segments


class ContinuationTests(unittest.TestCase):
    def test_repeated_edges_in_every_bin_support_reference_without_moving_it(self):
        road,segments=fixture();original=road['reference_line'];line,review=support_gaps(road,segments)
        self.assertEqual(line['unsupported_intervals_m'],[]);self.assertEqual(original['unsupported_intervals_m'],[[40,100]])
        self.assertEqual(line['points_W_xy'],original['points_W_xy']);self.assertFalse(review['actors_used'])
        road['reference_line']=line;self.assertEqual(support_kind(road,70),'two_sided_camera_edges_inferred_reference')

    def test_one_missing_edge_bin_cannot_be_filled_by_distant_evidence(self):
        road,segments=fixture();segments=[r for r in segments if not (r['points_W_xy'][0][0]==66 and r['points_W_xy'][0][1]<0)]
        self.assertEqual(support_gaps(road,segments)[0]['unsupported_intervals_m'],[[40,100]])

    def test_duplicate_lines_from_one_source_frame_are_not_independent_support(self):
        road,segments=fixture()
        for r in segments:r['frame_id']='one-frame'
        self.assertEqual(support_gaps(road,segments*5)[0]['unsupported_intervals_m'],[[40,100]])

    def test_no_automatic_continuation_across_junction_or_multiple_lanes(self):
        road,segments=fixture();road['junctions']=[dict(road_station_range_m=[60,80])]
        self.assertEqual(support_gaps(road,segments)[0]['unsupported_intervals_m'],[[40,100]])
        road['junctions']=[];road['boundaries_y']=[1.75,-1.75,-5.25]
        self.assertEqual(support_gaps(road,segments)[0]['unsupported_intervals_m'],[[40,100]])
