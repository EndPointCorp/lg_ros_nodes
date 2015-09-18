#!/usr/bin/env python

import rospy
import rostopic
from threading import Lock
from std_msgs.msg import String
from lg_common import StateChanger
from lg_common.msg import StringArray
from lg_common.msg import ApplicationState


def main():
    rospy.init_node('state_handler')

    state_changer = StateChanger()
    rospy.Subscriber('/state_handler/activate', StringArray, state_changer.locked_state_handler)

    rospy.spin()


if __name__ == '__main__':
    main()
