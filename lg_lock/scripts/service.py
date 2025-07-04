#!/usr/bin/env python3
import rospy

from std_srvs.srv import SetBool
from lg_lock import LockerService

from lg_msg_defs.srv import IsLocked, Lock, UnLock
from lg_msg_defs.msg import LockState

from lg_common.helpers import run_with_influx_exception_handler

NODE_NAME = 'lg_lock'
from lg_common.logger import get_logger
logger = get_logger(NODE_NAME)


def init():
    rospy.init_node(NODE_NAME, anonymous=False)

    password = rospy.get_param('~password', None)
    suppress_spacenav = rospy.get_param('~suppress_spacenav', True)
    locked = rospy.get_param('~locked', False)

    statePublisher = rospy.Publisher('/lg_lock/locked', LockState, queue_size=1, latch=True)

    if not password:
        logger.error('No or blank password provided, exiting...')
        print("No or blank password provided, exiting...")
        return

    suppressProxy = rospy.ServiceProxy('/spacenav_wrapper/suppress', SetBool, persistent=False)

    def onChange(state):
        if suppress_spacenav:
            logger.debug("Suppress spacenav: {}".format(state))
            suppressProxy(state)

    service = LockerService(statePublisher, password, locked, onChange)

    rospy.Service('/lg_lock/is_locked', IsLocked, service.get_state)
    rospy.Service('/lg_lock/lock', Lock, service.lock)
    rospy.Service('/lg_lock/unlock', UnLock, service.unlock)
    rospy.spin()


if __name__ == "__main__":
    run_with_influx_exception_handler(init, NODE_NAME)
