import unittest
import numpy as np
from core.driving_scene import fleet,asset_geometry_bounds,actor_mesh_transform
from core.scene_fusion import dimensions


class ActorGeometryTests(unittest.TestCase):
    def test_rendered_opaque_and_ghost_assets_share_placement_dimensions(self):
        for kind in ('car','suv','truck','bus','person','bicycle','motorcycle'):
            for ghost in (False,True):
                with self.subTest(kind=kind,ghost=ghost):
                    asset=fleet(kind,ghost);size=dimensions(kind)
                    origin,scale=actor_mesh_transform(asset,size)
                    lo,hi=asset_geometry_bounds(asset)
                    lo=(lo-origin)*scale;hi=(hi-origin)*scale
                    np.testing.assert_allclose(hi-lo,[size[0],size[2],size[1]],atol=1e-7)
                    self.assertAlmostEqual(lo[1],0.)
                    np.testing.assert_allclose((hi+lo)[[0,2]],0.,atol=1e-7)
