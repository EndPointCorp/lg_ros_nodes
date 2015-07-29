#!/usr/bin/env python

import rospy
from lg_media import MediaService
from lg_media import ROS_NODE_NAME
from lg_media import DEFAULT_VIEWPORT
from lg_media.msg import AdhocMedias


def main():
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    media_service = MediaService()
    # TODO
    # there will be multiple topics, multiple viewports
    # as the desired number of instances (viewports) will
    # be defined by the launch file
    topic_name = ("/media_service/" +
                  rospy.get_param("~viewport", DEFAULT_VIEWPORT))
    rospy.Subscriber(topic_name, AdhocMedias, media_service.listener)
    rospy.loginfo(ROS_NODE_NAME + " has started.")
    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4