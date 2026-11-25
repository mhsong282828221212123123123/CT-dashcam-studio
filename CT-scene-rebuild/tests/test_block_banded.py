import unittest
import numpy as np
from ct_scene.block_banded import solve


class BlockSolverTests(unittest.TestCase):
    def test_matches_dense_solution_and_covariance_with_rotated_measurements(self):
        rng=np.random.default_rng(91)
        for n in [1,2,3,9,31]:
            L=np.eye(2*n)
            for i in range(n):
                if i>=1:L[2*i:2*i+2,2*i-2:2*i]=rng.normal(size=(2,2))*.3
                if i>=2:L[2*i:2*i+2,2*i-4:2*i-2]=rng.normal(size=(2,2))*.3
            A=L@L.T+np.eye(2*n)*.1;b=rng.normal(size=(n,2))
            d=np.array([A[2*i:2*i+2,2*i:2*i+2] for i in range(n)]);lo1=np.zeros_like(d);lo2=np.zeros_like(d)
            for i in range(1,n):lo1[i]=A[2*i:2*i+2,2*i-2:2*i]
            for i in range(2,n):lo2[i]=A[2*i:2*i+2,2*i-4:2*i-2]
            x,cov=solve(d,lo1,lo2,b,True);inv=np.linalg.inv(A)
            np.testing.assert_allclose(x,np.linalg.solve(A,b.ravel()).reshape(n,2),atol=1e-10)
            np.testing.assert_allclose(cov,[inv[2*i:2*i+2,2*i:2*i+2] for i in range(n)],atol=1e-10)
    def test_singular_input_is_rejected(self):
        a=np.zeros((2,2,2))
        with self.assertRaises(ValueError):solve(a,a,a,np.zeros((2,2)))
