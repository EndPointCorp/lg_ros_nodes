#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

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
    router = GlobePoseRouter(
        selected=rospy.get_param('~default_base', 'earth'),
        command_outputs={base: pub.publish
                         for base, pub in command_pubs.items()},
        sync_outputs={base: pub.publish for base, pub in sync_pubs.items()},
        pose_output=pose_pub.publish,
    )

    def select(message):
        try:
            router.select(message.data)
        except ValueError as exc:
            rospy.logerr(str(exc))

    rospy.Subscriber('/base/selected', String, select, queue_size=1)
    rospy.Subscriber('/touchscreen/owner', String,
                     lambda msg: router.set_owner(msg.data), queue_size=1)
    rospy.Subscriber('/globe/control/pose', PoseStamped,
                     router.handle_command, queue_size=1)
    for base in BASES:
        rospy.Subscriber(
            '/{}/pose'.format(base), PoseStamped,
            lambda msg, source=base: router.handle_feedback(source, msg),
            queue_size=1,
        )

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
