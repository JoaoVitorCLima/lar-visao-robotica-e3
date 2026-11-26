import math
import unittest

from multi_camera_robot_localization.geometry import fuse_planar_poses, wrap


class GeometryTest(unittest.TestCase):
    def test_wraps_angle(self):
        self.assertAlmostEqual(wrap(math.radians(181)), math.radians(-179))

    def test_fuses_position_and_circular_yaw(self):
        result = fuse_planar_poses([
            (1.0, 2.0, math.radians(179)),
            (3.0, 4.0, math.radians(-179)),
        ])
        self.assertAlmostEqual(result[0], 2.0)
        self.assertAlmostEqual(result[1], 3.0)
        self.assertAlmostEqual(abs(result[2]), math.pi, places=6)

    def test_rejects_invalid_weights(self):
        with self.assertRaises(ValueError):
            fuse_planar_poses([(0.0, 0.0, 0.0)], [0.0])


if __name__ == '__main__':
    unittest.main()
