#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from lg_common.srv import USCSMessage, USCSMessageResponse
from interactivespaces_msgs.msg import GenericMessage


class USCSState:

    def __init__(self):
        self.state = None

    def send_uscs_message(self, req):
        rospy.loginfo("Sending message {}".format(self.state))
        if self.state:
            return USCSMessageResponse(self.state.type, self.state.message)
        return USCSMessageResponse()

    def update_uscs_message(self, message):
        rospy.loginfo("Getting message {}".format(message))
        self.state = message


def main():
    rospy.init_node('lg_uscs', anonymous=False)

    director_topic = rospy.get_param('~director_topic', '/director/scene')
    message_topic = rospy.get_param('~message_topic', '/uscs/message')

    us = USCSState()
    rospy.Subscriber(director_topic, GenericMessage, us.update_uscs_message)
    rospy.Service(message_topic, USCSMessage, us.send_uscs_message)

    rospy.spin()


if __name__ == "__main__":
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
