import unittest
from tools.prepare_scene_osm_api import convert
from core.scene_map import RoadMap


class OsmApiTests(unittest.TestCase):
    def test_shared_node_topology_survives_conversion_and_nonroads_are_ignored(self):
        raw='''<osm><node id="1" lat="37" lon="127"/>
        <node id="2" lat="37.001" lon="127"/><node id="3" lat="37.002" lon="127"/>
        <node id="4" lat="37.001" lon="127.001"/>
        <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/></way>
        <way id="11"><nd ref="2"/><nd ref="4"/><tag k="highway" v="residential"/></way>
        <way id="12"><nd ref="1"/><nd ref="4"/><tag k="building" v="yes"/></way></osm>'''
        payload=convert(raw)
        self.assertEqual(len(payload['elements']),2)
        roads=RoadMap(payload,[37,127])
        self.assertEqual(len(roads.junction_records),1)
        self.assertEqual(roads.junction_records[0]['node_id'],2)

    def test_missing_nodes_and_error_payloads_are_rejected(self):
        for raw in ['<osm><way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/></way></osm>',
                    '<osm><error>Too many nodes</error></osm>','<html/>']:
            with self.assertRaises(ValueError):convert(raw)
