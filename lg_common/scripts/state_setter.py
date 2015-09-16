#!/usr/bin/env python

import rospy
import rostopic
from threading import Lock
from std_msgs.msg import String
from lg_common.msg import ApplicationState


class StateChanger:
    """
    A class to handle state changes. It listens on it's own topic which receives
    a string (topic name to make active), then will grab all grab all topics with
    ApplicationState as their message type, assure that the desired active topic
    exists, and then make that topic VISIBLE while making all other topics HIDDEN
    """
    def __init__(self):
        self.pubbers = {}
        self.message_type_s = 'lg_common/ApplicationState'
        self.message_type = ApplicationState
        self.lock = Lock()

    def locked_state_handler(self, msg):
        """
        Locks the handle_state_change function

        msg: std_msgs/String
        """
        with self.lock:
            self.handle_state_change(msg)

    def handle_state_change(self, msg):
        active = msg.data
        # returns a list of topics w/ the state specified
        topics = rostopic.find_by_type(self.message_type_s)
        self.set_pubbers(topics)
        if active not in self.pubbers:
            rospy.logerr('Could not find the desired topic to set the state of')
            return
        for topic, pub in self.pubbers.iteritems():
            if topic == active:
                pub.publish(ApplicationState.VISIBLE)
            else:
                pub.publish(ApplicationState.HIDDEN)

    def set_pubbers(self, topics):
        """
        Keeps a list of publishers in self.pubbers, makes sure to
        reuse any existing publishers, throws away any old publishers
        that are not in the current topics

        topics: [String]
        """
        new_pubbers = {}
        for topic in topics:
            if topic in self.pubbers:
                new_pubbers[topic] = self.pubbers[topic]
            else:
                new_pubbers[topic] = rospy.Publisher(topic, self.message_type,
                                                     queue_size=10)
        self.pubbers = new_pubbers


def main():
    rospy.init_node('state_handler')

    state_changer = StateChanger()
    rospy.Subscriber('/state_handler/activate', String, state_changer.locked_state_handler)

    rospy.spin()


if __name__ == '__main__':
    main()
