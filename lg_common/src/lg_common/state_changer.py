from lg_common.msg import ApplicationState
from threading import Lock


class StateChanger:
    """
    A class to handle state changes. It listens on it's own topic which receives
    an array of strings (topic names to make active), then will grab all grab all
    topics with ApplicationState as their message type, assure that the desired
    active topics exists, and then make those topics VISIBLE while making all
    other topics HIDDEN
    """
    def __init__(self):
        self.pubbers = {}
        self.message_type_s = 'lg_common/ApplicationState'
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
        activities = [s.data for s in msg.strings]
        # returns a list of topics w/ the state specified
        topics = rostopic.find_by_type(self.message_type_s)
        self.set_pubbers(topics)
        for active in activities:
            if active not in self.pubbers:
                rospy.logerr('Could not find the desired topic (%s) to set the state of' % active)
        for topic, pub in self.pubbers.iteritems():
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
        new_pubbers = {}
        for topic in topics:
            if topic in self.pubbers:
                new_pubbers[topic] = self.pubbers[topic]
            else:
                new_pubbers[topic] = rospy.Publisher(topic, self.message_type,
                                                     queue_size=10)
        self.pubbers = new_pubbers
