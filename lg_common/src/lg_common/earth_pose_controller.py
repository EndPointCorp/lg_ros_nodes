import copy
import math
import time


EARTH_RADIUS_METERS = 6378100.0


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _angle_error(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


class EarthPoseController(object):
    """Drive Earth toward a streamed absolute pose using SpaceNavigator axes.

    Google Earth does not expose a real-time absolute-camera API. This small
    closed-loop controller turns the latest touchscreen target into the same
    Twist input already consumed by the virtual SpaceNavigator. Earth remains
    the source of truth: each correction is recalculated from ViewSync pose
    feedback instead of integrating commands locally.
    """

    def __init__(self, twist_output, twist_factory, clock=None,
                 target_ttl=0.25, axis_limit=0.7,
                 position_gain=2.0, zoom_gain=1.5,
                 angle_gain=1.5):
        self.twist_output = twist_output
        self.twist_factory = twist_factory
        self.clock = clock or time.monotonic
        self.target_ttl = target_ttl
        self.axis_limit = axis_limit
        self.position_gain = position_gain
        self.zoom_gain = zoom_gain
        self.angle_gain = angle_gain
        self.current_pose = None
        self.target_pose = None
        self.target_time = None
        self.moving = False

    def set_pose(self, pose):
        self.current_pose = copy.deepcopy(pose)

    def set_target(self, pose):
        self.target_pose = copy.deepcopy(pose)
        self.target_time = self.clock()

    def stop(self):
        self.target_pose = None
        self.target_time = None
        self._publish_stop()

    def tick(self, now=None):
        now = self.clock() if now is None else now
        if (self.current_pose is None or self.target_pose is None or
                now - self.target_time > self.target_ttl):
            self._publish_stop()
            return False

        current = self.current_pose
        target = self.target_pose
        altitude = max(abs(current.position.z), 80.0)

        north = (
            math.radians(target.position.y - current.position.y) *
            EARTH_RADIUS_METERS / altitude
        )
        east = (
            math.radians(_angle_error(target.position.x,
                                      current.position.x)) *
            EARTH_RADIUS_METERS *
            math.cos(math.radians(current.position.y)) / altitude
        )
        heading = math.radians(current.orientation.z)

        twist = self.twist_factory()
        twist.linear.x = _clamp(
            self.position_gain *
            (math.cos(heading) * north + math.sin(heading) * east),
            self.axis_limit,
        )
        twist.linear.y = _clamp(
            self.position_gain *
            (math.sin(heading) * north - math.cos(heading) * east),
            self.axis_limit,
        )
        twist.linear.z = _clamp(
            self.zoom_gain * math.log(
                max(target.position.z, 1.0) / altitude
            ),
            self.axis_limit,
        )
        # SpaceNavigator yaw is angular.z. Its tilt axis is angular.y; the
        # Cesium driver performs the same physical-axis swap on input.
        twist.angular.z = _clamp(
            -self.angle_gain * math.radians(
                _angle_error(target.orientation.z,
                             current.orientation.z)
            ),
            self.axis_limit,
        )
        twist.angular.y = _clamp(
            -self.angle_gain * math.radians(
                target.orientation.x - current.orientation.x
            ),
            self.axis_limit,
        )

        if self._is_zero(twist):
            self._publish_stop()
            return False

        self.twist_output(twist)
        self.moving = True
        return True

    def _publish_stop(self):
        if self.moving:
            self.twist_output(self.twist_factory())
            self.moving = False

    @staticmethod
    def _is_zero(twist):
        return (
            abs(twist.linear.x) < 0.001 and
            abs(twist.linear.y) < 0.001 and
            abs(twist.linear.z) < 0.001 and
            abs(twist.angular.y) < 0.001 and
            abs(twist.angular.z) < 0.001
        )
