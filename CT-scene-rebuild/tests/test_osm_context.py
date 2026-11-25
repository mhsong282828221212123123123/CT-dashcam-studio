import copy,unittest
from ct_scene.osm_context import junctions,candidates


def fixture():
    return [dict(id=1,nodes=[1,2,3],geometry=[[37,-.001],[37,0],[37,.001]],tags={'highway':'primary'}),
            dict(id=2,nodes=[4,2,5],geometry=[[36.999,0],[37,0],[37.001,0]],tags={'highway':'secondary'})]


class OSMTests(unittest.TestCase):
    def test_branch_bearings_are_geometry_not_lane_tags(self):
        row=junctions(fixture())[0]
        self.assertEqual(sorted(round(b['bearing_deg']) for b in row['branches']),[0,90,180,270])
        result=candidates(fixture(),[row],37,-.0001,90)['junction_candidates'][0]
        self.assertEqual(sorted(round(b['relative_bearing_deg']) for b in result['branches']),[-180,-90,0,90])

    def test_tangent_looks_past_tiny_adjacent_node(self):
        w=fixture();w[1]['nodes']=[4,2,6,5];w[1]['geometry']=[[36.999,0],[37,0],[37.000001,.000001],[37.001,0]]
        arm=next(b for b in junctions(w)[0]['branches'] if b['neighbor_node_id']==6)
        self.assertAlmostEqual(arm['bearing_deg'],0.,places=3)
    def test_shared_node_is_candidate_not_lane_measurement(self):
        ways=fixture();js=junctions(ways);self.assertEqual(len(js),1)
        r=candidates(ways,js,37,-.0001,90)
        self.assertEqual(len(r['junction_candidates']),1)
        self.assertFalse(js[0]['lane_information_used'])

    def test_grade_separated_crossing_is_not_junction(self):
        w=fixture();w[1]['tags'].update(layer='1',bridge='yes')
        self.assertEqual(junctions(w),[])

    def test_lane_and_width_tags_do_not_change_context(self):
        w=fixture();before=candidates(w,junctions(w),37,-.0001,90)
        changed=copy.deepcopy(w)
        for way in changed:way['tags'].update(lanes='99',width='400')
        self.assertEqual(before,candidates(changed,junctions(changed),37,-.0001,90))

    def test_missing_map_and_wrong_region_abstain(self):
        self.assertEqual(candidates([],[],37,0,0)['way_candidates'],[])
        self.assertEqual(candidates(fixture(),junctions(fixture()),38,1,90)['junction_candidates'],[])

    def test_geometric_cross_without_shared_node_is_not_junction(self):
        w=fixture();w[1]['nodes']=[4,6,5];self.assertEqual(junctions(w),[])

    def test_context_surface_extract_uses_direct_osm_path_when_cache_roots_empty(self):
        import tempfile
        from pathlib import Path
        from ct_scene.artifacts import canonical,digest
        from ct_scene.context_surface import extract
        xml='<osm version="0.6"><node id="1" lat="37.0" lon="127.0"/><node id="2" lat="37.001" lon="127.0"/><way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/></way></osm>'
        with tempfile.TemporaryDirectory() as td:
            osm=Path(td)/'sample.osm';osm.write_text(xml,encoding='utf-8')
            tel=Path(td)/'tel';tel.mkdir();(tel/'telemetry.jsonl').write_text('',encoding='utf-8')
            manifest=dict(config_sha256=digest(canonical(dict(osm_sha256=digest(osm.read_bytes())))))
            res=extract(manifest,tel,[0.,1.],[[0,0],[1,0]],[],cache_roots=[],osm_path=str(osm))
            self.assertNotEqual(res['status'],'matching_cached_OSM_unavailable')
