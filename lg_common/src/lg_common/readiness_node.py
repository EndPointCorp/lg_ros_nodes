import time
import rospy
import threading
import json
import threading

from lg_msg_defs.msg import AdhocBrowsers
from lg_msg_defs.msg import AdhocBrowser
from lg_msg_defs.msg import Ready
from std_msgs.msg import String

from lg_common.logger import get_logger
logger = get_logger('readiness_node')


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
        logger.debug("ReadinessHandbrake registered message")
        self._timer_thread = threading.Thread(
            target=self._timer_worker_thread
        )
        self._timer_thread.start()

    def _timer_worker_thread(self):
        for interval in range(0, self.timeout):
            logger.debug("Waiting for readiness %s" % interval)
            rospy.sleep(1)

        logger.info("Executing readiness handbrake to activate unactivated browsers within specified timeout of %s secs" % self.timeout)
        self.callback(force=True)


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
            logger.info("Received new director scene")
            self.ready = False
            message = json.loads(message.message)
            slug = message.get('slug', None)
            if slug:
                logger.info("Waiting for browsers to join to scene %s" % slug)
                self.uscs_messages[slug] = message
                self._purge_state(slug)
                logger.info("Current state is: %s" % json.dumps(self.state, indent=4))
            else:
                logger.warning("Readiness node received message without slug this no preloading will be performed")

    def aggregate_browser_instances(self, browsers):
        """
        Callback for common browser topic with AdhocBrowsers msg type

        Append browser instance names to state var
        Append only the browsers that have the `preload` flag set to true
        """

        for browser in browsers.browsers:
            if browser.preload and (browser.scene_slug == self.state['slug']):
                browser_id = browser.id
                if browser_id not in self.state['browsers']:
                    with self.lock:
                        logger.info("Browser with id %s added to waiting pool" % browser_id)
                        self.state['browsers'].append(browser_id)

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
            logger.warning("Could not get number of preloadable browsers for this USCS message for slug: %s" % slug)
            return num_browsers

        for window in windows:
            if window['activity'] == 'browser':
                activity_config = window.get('activity_config', None)
                if activity_config:
                    preload = activity_config.get('preload', None)
                    if preload:
                        num_browsers += 1

        logger.debug("Got number of preloadable browsers = %s" % num_browsers)
        return num_browsers

    def _all_browsers_joined(self):
        number_of_browsers_to_join = self._get_number_of_prelaodable_browsers_to_join()
        ready_browsers = set(self.state['ready_browsers'])
        registered_browsers = self.state['browsers']

        if len(ready_browsers) == number_of_browsers_to_join:
            if set(ready_browsers) == set(registered_browsers):
                return True
            else:
                return False
        else:
            logger.debug("Not enough browsers gathered - joined: %s, registered %s, total: %s" % (ready_browsers, registered_browsers, number_of_browsers_to_join))
            return False

    def _publish_readiness(self, force=False):
        """
        Emit a message that will let adhoc browser pool know that we're ready
        if readiness publication is forced, make all browsers ready
        """
        ready_msg = Ready()
        ready_msg.scene_slug = self.state['slug']
        if force:
            self.state['ready_browsers'] = self.state['browsers']
        ready_msg.instances = self.state['ready_browsers']
        ready_msg.activity_type = 'browser'
        if ready_msg.instances:
            logger.info("Became ready with %s browsers (force=%s)" % (ready_msg.instances, force))
            self.readiness_publisher.publish(ready_msg)
        else:
            logger.warning("Prevented emitting readiness message with empty instances list %s" % ready_msg)

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
            logger.debug("Got instance name ready signal from %s" % instance_name)
            if instance_name in self.state['browsers']:
                if instance_name not in self.state['ready_browsers']:
                    self.state['ready_browsers'].append(instance_name)
                    logger.info("Browser with ID %s joined (%s out of %s)" % (instance_name, len(self.state['ready_browsers']), len(self.state['browsers'])))
                    logger.info("Current state is: %s" % json.dumps(self.state, indent=4))
                else:
                    logger.debug("Readiness node received browser instance id that was already ready")
            else:
                logger.debug("Readiness node received unknown browser instance id")

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
            logger.info("Scene %s is becoming ready (force=%s)" % (self.state['slug'], force))
            if force:
                logger.warning("Readiness was forced")
            self._publish_readiness(force=force)

    def try_to_become_ready(self, force=False):
        """
        Method for becoming ready e.g. publishing readiness
        """
        if self._all_browsers_joined() and not self.ready:
            self.become_ready()

        if force is True and self.ready is not True:
            message = "Scene %s did not become ready within specified timeout" % self.state['slug']
            logger.warning(message)
            self.timeout_publisher.publish(String(data=message))
            self.become_ready(force=force)
