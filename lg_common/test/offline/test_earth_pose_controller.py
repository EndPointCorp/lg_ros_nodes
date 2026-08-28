import importlib.util
import os
import unittest


MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'src', 'lg_common',
    'earth_pose_controller.py')
SPEC = importlib.util.spec_from_file_location('earth_pose_controller',
                                               MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
EarthPoseController = MODULE.EarthPoseController


class Vector(object):
    def __init__(self):
        self.x = self.y = self.z = self.w = 0.0


class Pose(object):
    def __init__(self):
        self.position = Vector()
        self.orientation = Vector()


class Twist(object):
    def __init__(self):
        self.linear = Vector()
        self.angular = Vector()


def pose(lon=0.0, lat=0.0, altitude=1000.0, heading=0.0, tilt=0.0):
    value = Pose()
    value.position.x = lon
    value.position.y = lat
    value.position.z = altitude
    value.orientation.z = heading
    value.orientation.x = tilt
    return value


class TestEarthPoseController(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.output = []
        self.controller = EarthPoseController(
            self.output.append, Twist, clock=lambda: self.now)
        self.controller.set_pose(pose())

    def test_pans_toward_target_in_camera_axes(self):
        self.controller.set_target(pose(lat=0.001))
        self.assertTrue(self.controller.tick())
        self.assertGreater(self.output[-1].linear.x, 0)

        self.controller.set_pose(pose(heading=90.0))
        self.controller.set_target(pose(lat=0.001, heading=90.0))
        self.controller.tick()
        self.assertGreater(self.output[-1].linear.y, 0)

    def test_uses_shortest_heading_and_longitude_paths(self):
        self.controller.set_pose(pose(lon=179.9, heading=359.0))
        self.controller.set_target(pose(lon=-179.9, heading=1.0))
        self.controller.tick()
        self.assertLess(self.output[-1].linear.y, 0)
        self.assertLess(self.output[-1].angular.z, 0)

    def test_stale_target_publishes_one_stop(self):
        self.controller.set_target(pose(lat=0.001))
        self.controller.tick()
        self.now += 0.3
        self.assertFalse(self.controller.tick())
        self.assertEqual(0.0, self.output[-1].linear.x)
        count = len(self.output)
        self.controller.tick()
        self.assertEqual(count, len(self.output))

    def test_stop_clears_active_target(self):
        self.controller.set_target(pose(altitude=2000.0))
        self.controller.tick()
        self.controller.stop()
        self.assertFalse(self.controller.tick())
        self.assertEqual(0.0, self.output[-1].linear.z)


if __name__ == '__main__':
    unittest.main()
