import unittest

import numpy as np

from multi_camera_robot_localization.geometry import triangulate_rays


class TriangulationTest(unittest.TestCase):
    def test_intersects_two_positive_rays(self):
        point, residual = triangulate_rays(
            [0.0, 0.0, 2.0], [0.5, 0.0, -1.0],
            [1.0, 0.0, 2.0], [-0.5, 0.0, -1.0],
        )
        self.assertTrue(np.allclose(point, [0.5, 0.0, 1.0], atol=1e-6))
        self.assertLess(residual, 1e-6)

    def test_rejects_point_behind_camera(self):
        with self.assertRaises(ValueError):
            triangulate_rays(
                [0.0, 0.0, 0.0], [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0], [0.0, 0.0, 1.0],
            )


if __name__ == '__main__':
    unittest.main()
