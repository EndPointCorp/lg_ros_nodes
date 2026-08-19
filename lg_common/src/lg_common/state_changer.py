from lg_msg_defs.msg import ApplicationState
from threading import Lock
import rostopic
import rospy

from lg_common.logger import get_logger
logger = get_logger('state_changer')


class StateChanger:
    """
    A class to handle state changes. It listens on it's own topic which receives
    an array of strings (topic names to make active), then will grab all grab all
    topics with ApplicationState as their message type, assure that the desired
    active topics exists, and then make those topics VISIBLE while making all
    other topics HIDDEN
    """
    def __init__(self, protected_topics=None):
        self.pubbers = {}
        # Base visibility has one owner. Generic activity changes may still
        # hide/show overlays, but cannot blank or switch the globe underneath.
        self.protected_topics = set(protected_topics or (
            '/earth/state', '/cesium/state', '/unreal/state'))
        self.message_type_s = 'lg_msg_defs/ApplicationState'
        self.message_type = ApplicationState
        self.lock = Lock()

    def locked_state_handler(self, msg):
        """
        Locks the handle_state_change function

        msg: lg_common/StringArray
        """
        with self.lock:
            self.handle_state_change(msg)

    def handle_state_change(self, msg):
        activities = msg.strings
        # returns a list of topics w/ the state specified
        topics = [topic for topic in rostopic.find_by_type(self.message_type_s)
                  if topic not in self.protected_topics]
        self.set_pubbers(topics)
        for active in activities:
            if active not in self.pubbers:
                logger.error('Could not find the desired topic (%s) to set the state of' % active)
        for topic, pub in self.pubbers.items():
            if topic in activities:
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
        new = False
        new_pubbers = {}
        for topic in topics:
            if topic in self.pubbers:
                new_pubbers[topic] = self.pubbers[topic]
            else:
                new = True
                new_pubbers[topic] = rospy.Publisher(topic, self.message_type,
                                                     queue_size=10)
        self.pubbers = new_pubbers
        if new:
            rospy.sleep(1)
