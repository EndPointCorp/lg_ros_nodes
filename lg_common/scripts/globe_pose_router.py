#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import String

from lg_common.earth_pose_controller import EarthPoseController
from lg_common.globe_pose_router import BASES, GlobePoseRouter
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'globe_pose_router'


def main():
    rospy.init_node(NODE_NAME)

    command_pubs = {
        'earth': rospy.Publisher('/earth/query/flyto_pose_camera', Pose,
                                 queue_size=1),
        'cesium': rospy.Publisher('/cesium/control/pose', Pose, queue_size=1),
        'unreal': rospy.Publisher('/unreal/control/pose', Pose, queue_size=1),
    }
    sync_pubs = {
        'earth': rospy.Publisher('/earth/query/sync_pose_camera', Pose,
                                 queue_size=1),
        'cesium': rospy.Publisher('/cesium/sync/pose', Pose, queue_size=1),
        'unreal': rospy.Publisher('/unreal/sync/pose', Pose, queue_size=1),
    }
    pose_pub = rospy.Publisher('/globe/pose', PoseStamped, latch=True,
                               queue_size=1)
    earth_twist_pub = rospy.Publisher('/navtransform/twist', Twist,
                                      queue_size=1)
    earth_controller = EarthPoseController(
        twist_output=earth_twist_pub.publish,
        twist_factory=Twist,
        target_ttl=float(rospy.get_param('~earth_target_ttl', 2.0)),
        axis_limit=float(rospy.get_param('~earth_axis_limit', 0.7)),
        position_gain=float(rospy.get_param('~earth_position_gain', 2.0)),
        zoom_gain=float(rospy.get_param('~earth_zoom_gain', 1.5)),
        angle_gain=float(rospy.get_param('~earth_angle_gain', 1.5)),
    )
    router = GlobePoseRouter(
        selected=rospy.get_param('~default_base', 'earth'),
        command_outputs={base: pub.publish
                         for base, pub in command_pubs.items()},
        live_outputs={
            'earth': earth_controller.set_target,
            'cesium': command_pubs['cesium'].publish,
            'unreal': command_pubs['unreal'].publish,
        },
        live_stop_outputs={'earth': earth_controller.stop},
        sync_outputs={base: pub.publish for base, pub in sync_pubs.items()},
        pose_output=pose_pub.publish,
        session_order_grace=float(
            rospy.get_param('~session_order_grace', 0.1)
        ),
    )

    def select(message):
        try:
            router.select(message.data)
        except ValueError as exc:
            rospy.logerr(str(exc))

    rospy.Subscriber('/base/selected', String, select, queue_size=1)
    rospy.Subscriber('/touchscreen/owner', String,
                     lambda msg: router.set_owner(msg.data), queue_size=1)
    rospy.Subscriber('/globe/control/session', String,
                     router.handle_control_session, queue_size=1)
    rospy.Subscriber('/globe/control/pose', PoseStamped,
                     router.handle_command, queue_size=1)
    for base in BASES:
        def feedback(message, source=base):
            if router.handle_feedback(source, message) and source == 'earth':
                earth_controller.set_pose(router.latest_pose.pose)

        rospy.Subscriber(
            '/{}/pose'.format(base), PoseStamped,
            feedback,
            queue_size=1,
        )

    background_interval = float(
        rospy.get_param('~background_sync_interval', 0.1)
    )
    rospy.Timer(rospy.Duration(background_interval),
                lambda _event: router.sync_inactive())
    earth_control_rate = float(rospy.get_param('~earth_control_rate', 60.0))
    rospy.Timer(rospy.Duration(1.0 / earth_control_rate),
                lambda _event: earth_controller.tick())

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
