import time
import rospy
import threading
import json
import threading

from lg_common.msg import AdhocBrowsers
from lg_common.msg import AdhocBrowser
from lg_common.msg import Ready
from std_msgs.msg import String


class ReadinessHandbrake(object):
    """
    Makes readiness node become ready within specified timeout
    """
    def __init__(self, callback, timeout):
        self.callback = callback
        self.timeout = timeout

    def handle_uscs_message(self, message):
        """
        Reset the timer
        """
        rospy.loginfo("ReadinessHandbrake registered message")
        self._timer_thread = threading.Thread(
            target=self._timer_worker_thread
        )
        self._timer_thread.start()

    def _timer_worker_thread(self):
        for interval in range(0, self.timeout):
            rospy.logdebug("Waiting for readiness %s" % interval)
            rospy.sleep(1)

        self.callback(force=True)
        rospy.loginfo("Executing readiness handbrake callback")


class ReadinessNode(object):
    """
    Keeps track of list of browsers indexed by scene slugs that they belong to
    Emits a message once all browsers have checked in on a topic

    self.state = {
                    'slug': 'scene-test',
                    'ready_browsers': []
                    'browsers': [
                        'adhoc_browser_f9db1u3b4',
                        'adhoc_browser_23048h5ue'
                    ]
                 }
    """
    def __init__(self,
                 readiness_publisher=None,
                 timeout_publisher=None
                 ):
        self.lock = threading.Lock()
        self._purge_state(None)
        self.ready = False
        self.last_slug = None
        self.uscs_messages = {}
        self.readiness_publisher = readiness_publisher
        self.timeout_publisher = timeout_publisher

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
        self.state = {'slug': slug,
                      'browsers': [],
                      'ready_browsers': []}

    def save_uscs_message(self, message):
        """
        Saves director message scene to know
        how many browsers should be activated for this scene

        """
        with self.lock:
            self.ready = False
            message = json.loads(message.message)
            slug = message.get('slug', None)
            if slug:
                rospy.loginfo("Waiting for browsers to join to scene %s" % slug)
                self.uscs_messages[slug] = message
                self._purge_state(slug)

    def aggregate_browser_instances(self, message):
        """
        Callback for common browser topic with AdhocBrowsers msg type

        Append browser instance names to state var
        Append only the browsers that have the `preload` flag set to true
        """

        self.last_slug = self.state.get('slug', None)

        if message.scene_slug != self.last_slug:
            with self.lock:
                self._purge_state(message.scene_slug)

        for browser in message.browsers:
            if browser.preload:
                browser_id = browser.id
                if browser_id not in self.state['browsers']:
                    with self.lock:
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
            return num_browsers

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

    def _publish_readiness(self, force=False):
        if self.state['ready_browsers']:
            ready_msg = Ready()
            ready_msg.scene_slug = self.state['slug']
            if force:
                ready_msg.instances = self.state['browsers']
            else:
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

        self.try_to_become_ready()

    def become_ready(self, force=False):
        """
        Sets the effective state to "ready" by emitting messages
        and setting states.

        If force is used - all browsers (even these that are not ready)
        are going to be unhidden
        """
        with self.lock:
            self.ready = True
            rospy.loginfo("Scene %s is becoming active" % self.state['slug'])
            self._publish_readiness(force=force)
            self._purge_state(self.last_slug)

    def try_to_become_ready(self, force=False):
        """
        Method for becoming ready e.g. publishing readiness
        """
        if self._ready():
            self.become_ready()

        if force is True and self.ready is not True:
            message = "Scene %s did not become ready within specified timeout" % self.state['slug']
            rospy.logwarn(message)
            self.timeout_publisher.publish(String(data=message))
            self.become_ready(force=force)
