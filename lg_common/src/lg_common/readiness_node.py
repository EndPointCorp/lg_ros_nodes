import time
import rospy
import threading
import json

from lg_common.msg import AdhocBrowsers
from lg_common.msg import AdhocBrowser
from lg_common.msg import Ready


class ReadinessNode(object):
    """
    Keeps track of list of browsers indexed by scene slugs that they belong to
    Emits a message once all browsers have checked in on a topic

    self.state = {
                    'slug': 'scene-test',
                    'ready_browsers': []
                    'num_browsers': 0,
                    'browsers': [
                        'adhoc_browser_f9db1u3b4',
                        'adhoc_browser_23048h5ue'
                    ],
                    'timestamp': 1466167808.940136
                 }
    """
    def __init__(self,
                 readiness_publisher
                 ):
        self.lock = threading.Lock()
        self._purge_state(None)
        self.last_slug = None
        self.uscs_messages = {}
        self.readiness_publisher = readiness_publisher

    def node_ready(self, query):
        """
        Since director bridge should start running AFTER
        readiness node gets activated, provide a service
        that will become available after all subs and pubs
        get initialized to signal director bridge that
        readiness node is ready.
        """
        if self.lock and self.state:
            return True
        else:
            return False

    def _purge_state(self, slug):
        """
        Purges the state and sets the most up to date slug
        """
        with self.lock:
            self.state = {'slug': slug,
                          'num_browsers': 0,
                          'browsers': [],
                          'ready_browsers': [],
                          'timestamp': time.time()}

    def save_uscs_message(self, message):
        """
        Saves director message scene to know
        how many browsers should be activated for this scene

        """
        with self.lock:
            rospy.loginfo("Got uscs message: %s" % message)
            message = json.loads(message.message)
            slug = message.get('slug', None)

            if slug:
                self.uscs_messages[slug] = message

    def aggregate_browser_instances(self, message):
        """
        Callback for common browser topic with AdhocBrowsers msg type

        Append browser instance names to state var
        Append only the browsers that have the `preload` flag set to true
        """

        self.last_slug = self.state.get('slug', None)

        if message.scene_slug != self.last_slug:
            self._purge_state(message.scene_slug)

        for browser in message.browsers:
            if browser.preload:
                browser_id = browser.id
                if browser_id not in self.state['browsers']:
                    self.state['browsers'].append(browser_id)

        rospy.loginfo("Gathered new browsers: %s" % self.state)

    def _get_number_of_prelaodable_browsers_to_join(self):
        """
        Parses last USCS message and returns the number of preloadable
        browsers that should join the scene
        """
        num_browsers = 0

        try:
            windows = self.uscs_messages[self.state['slug']]['windows']
        except KeyError:
            slug = self.state['slug']
            rospy.logwarn("Could not get number of preloadable browsers for this USCS message for slug: %s" % slug)
            return 0

        for window in windows:
            if window['activity'] == 'browser':
                activity_config = window.get('activity_config', None)
                if activity_config:
                    preload = activity_config.get('preload', None)
                    if preload:
                        num_browsers += 1

        rospy.logdebug("Got number of preloadable browsers = %s" % num_browsers)
        return num_browsers

    def _ready(self):
        number_of_browsers_to_join = self._get_number_of_prelaodable_browsers_to_join()
        ready_browsers = set(self.state['ready_browsers'])
        registered_browsers = self.state['browsers']

        if len(ready_browsers) == number_of_browsers_to_join:
            if set(ready_browsers) == set(registered_browsers):
                return True
            else:
                return False
        else:
            rospy.logdebug("Not enough browsers gathered - joined: %s, registered %s, total: %s" % (ready_browsers, registered_browsers, number_of_browsers_to_join))
            return False

    def _publish_readiness(self):
        if self.state['ready_browsers']:
            ready_msg = Ready()
            ready_msg.scene_slug = self.state['slug']
            ready_msg.instances = self.state['ready_browsers']
            ready_msg.activity_type = 'browser'
            rospy.loginfo("Became ready: %s" % self.state)
            self.readiness_publisher.publish(ready_msg)
        else:
            rospy.logdebug("Tried to emit a /director/ready message without browser instnces")

    def handle_readiness(self, message):
        """
        Wait for browser extensions to check in under /director/window/ready
        Emit a /director/ready message as soon as all browsers that checked
        in under /browser_service/browsers got ready

        The number of browsers needs to match the number of preloadable
        browers in the director scene message
        """
        if message.data:
            instance_name = message.data
            rospy.logdebug("Got instance name ready signal from %s" % instance_name)
            if instance_name in self.state['browsers']:
                if instance_name not in self.state['ready_browsers']:
                    self.state['ready_browsers'].append(instance_name)
                    rospy.loginfo("State after one of the browsers became ready %s" % self.state)
                else:
                    rospy.logdebug("Readiness node received browser instance id that was already ready")
            else:
                rospy.logdebug("Readiness node received unknown browser instance id")

        if self._ready():
            self._publish_readiness()
            self._purge_state(self.last_slug)
