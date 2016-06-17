from lg_common.msg import AdhocBrowsers
from lg_common.msg import AdhocBrowser
from lg_common.msg import Ready
import time
import rospy

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
                    ],
                    'timestamp': 1466167808.940136
                 }
    """
    def __init__(self,
                 readiness_publisher):
        self._purge_state(None)
        self.readiness_publisher = readiness_publisher

    def _purge_state(self, slug):
        """
        Purges the state and sets the most up to date slug
        """
        self.state = { 'slug': slug,
                       'browsers': [],
                       'ready_browsers': [],
                       'timestamp': time.time()
                     }

    def aggregate_browser_instances(self, message):
        """
        Append browser instance names to state var
        Purge the state if new director message was published -
        rely on scene slug.
        """
        incoming_slug = message.scene_slug
        last_slug = self.state.get('slug', None)

        if incoming_slug != last_slug:
            # TODO (wz): what if slug is blank for consecutive scenes?
            self._purge_state(incoming_slug)

        for browser in message.browsers:
            # TODO (wz): prolly handle exceptions here
            # 'right_one_ea7ef35f'
            # browser.id - eassdsada
            # ready      - adhoc__left_one_eassdsada
            browser_id = browser.id
            self.state['browsers'].append(browser_id)

        rospy.loginfo("Gathered state: %s" % self.state)

    def _ready(self):
        if set(self.state['ready_browsers']) == set(self.state['browsers']):
            return True
        else:
            return False

    def _publish_readiness(self):
        ready_msg = Ready()
        ready_msg.scene_slug = self.state['slug']
        ready_msg.instances = self.state['ready_browsers']
        ready_msg.activity_type = 'browser'
        rospy.loginfo("Became ready: %s" % self.state)
        self.readiness_publisher.publish(ready_msg)

    def handle_readiness(self, message):
        """
        """
        if message.data:
            instance_name = message.data
            rospy.loginfo("Got instance name ready signal from %s" % instance_name)
            if instance_name in self.state['browsers']:
                self.state['ready_browsers'].append(instance_name)
                rospy.loginfo("State after one of the browsers became ready %s" % self.state)
            else:
                rospy.logwarn("Readiness node received unknown browser instance id")

        if self._ready():
            self._publish_readiness()
