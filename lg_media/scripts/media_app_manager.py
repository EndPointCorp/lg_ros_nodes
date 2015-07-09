#!/usr/bin/env python
"""
LG video ROS node manager.

1) explore ManagedApplication
2) will be handled by appctl (check the ticket #6)

receive video url from message

define video playing commands (String),
http://www.mplayerhq.hu/DOCS/HTML/en/control.html
http://www.mplayerhq.hu/DOCS/tech/slave.txt
http://stackoverflow.com/questions/4976276/is-it-possible-to-control-mplayer-from-another-program-easily

Considering vlc - it does have a control interface too.

notes:
    rospy.logdebug()
    while not rospy.is_shutdown() ...
    timeout = rospy.get_param('~timeout', 1)
    rospy.ROSInterruptException:
    rospy.service.ServiceException:

catkin package set up:
    catkin_create_pkg lg_media std_msgs rospy


catking_make
source catkin/devel/setup.bash
roslaunch lg_media/launch/dev.launch
rostopic list
rostopic pub --once /lg_media/presence std_msgs/Bool -- True

"""


import os

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool


ROS_NODE_NAME = "lg_media"


def presenter(data):
    """
    Start / quit video player process.

    """
    # TODO:
    # add argument - the URL of the video to play (both local and online)
    rospy.loginfo("called, data: %s" % data)
    # config_file = rospy.get_param(CONFIG_FILE, DEFAULT_CONFIG_FILE)
    #rospy.loginfo("%s using '%s' conf file." % (NODE_NAME, config_file))
    #rospy.logdebug()


def controller():
    """
    Control the video player application according to received commands.

    """
    pass


def listener():
    rospy.init_node(ROS_NODE_NAME)
    rospy.Subscriber("/lg_media/presence", Bool, presenter)
    rospy.Subscriber("/lg_media/control", String, controller)
    rospy.loginfo("lg_media started.")
    rospy.spin()


if __name__ == "__main__":
    listener()


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4