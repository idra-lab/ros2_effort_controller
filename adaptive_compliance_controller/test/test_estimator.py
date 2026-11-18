import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('estimate_tissue', Path(__file__).parents[1] / 'scripts/estimate_tissue.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TissueEstimate(unittest.TestCase):
    def test_recovers_hunt_crossley_coefficients(self):
        t = np.linspace(0., 8., 401)
        p = .004 + .002 * np.sin(t)
        v = .002 * np.cos(t)
        f = (12000. + 8000. * v) * p**1.35
        result = module.estimate(p, v, f)
        self.assertAlmostEqual(result['kappa'], 12000., places=7)
        self.assertAlmostEqual(result['viscosity'], 8000., places=6)
        self.assertLess(result['rmse_n'], 1e-12)

    def test_pure_elastic_samples_keep_nonnegative_viscosity(self):
        p = np.linspace(.001, .01, 100)
        v = np.sin(np.linspace(0, 8, 100))*.001
        result = module.estimate(p, v, 1000.*p**1.35)
        self.assertGreaterEqual(result['viscosity'], 0.)
        self.assertAlmostEqual(result['kappa'], 1000., places=7)

    def test_zero_excitation_is_not_identifiable(self):
        with self.assertRaises(ValueError):
            module.estimate([.001,.002,.003], [0.,0.,0.], [1.,2.,3.])

    def test_invalid_contact_and_shapes(self):
        for p, v, f in [([0.,.01,.02], [1.,2.,3.], [1.,2.,3.]),
                        ([.01,.02,.03], [1.,2.,3.], [1.,-1.,3.]),
                        ([.01,.02,.03], [1.,2.], [1.,2.,3.]),
                        ([.01,.02,np.nan], [1.,2.,3.], [1.,2.,3.])]:
            with self.assertRaises(ValueError):
                module.estimate(p, v, f)


if __name__ == '__main__':
    unittest.main()
