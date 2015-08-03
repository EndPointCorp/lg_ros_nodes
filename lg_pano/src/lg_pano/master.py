import rospy

from lg_common.msg import ApplicationState
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

SCALE = 100

class PanoMaster:
    def __init__(self, pov, pano, scale=SCALE):
        self.pov_publisher  = pov
        self.pano_publisher = pano
        self.pov = Quaternion()
        self.pano_url = String("")
        self.state = True
        self.scale = scale

    def handle_spacenav_msg(self, msg):
        """
        Changes the point of view based on incoming message

        ignores if state is False or the message has all zeros
        """
        if not self.state:
            return
        if (msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            return 
        self.pov.x = clamp(self.pov.x + msg.angular.x / self.scale, -1, 1)
        self.pov.y = clamp(self.pov.z + msg.angular.y / self.scale, -1, 1)
        self.pov.z = clamp(self.pov.z + msg.angular.z / self.scale, -1, 1)
        self.send_pov()

    def send_pano(self):
        self.pano_publisher.publish(self.pano_url)

    def send_pov(self):
        self.pov_publisher.publish(self.pov)

    def handle_state_msg(self, app_state):
        """
        Sets state to true, if visible
        """
        self.state = (app_state.state == ApplicationState.VISIBLE)

    def handle_init_msg(self, msg):
        # A new instance has connected; send it the current pano and pov
        # It doesn't really matter what the new instance sends; the presence of
        # any message tells us to send these messages.
        rospy.loginfo('Received init msg: %s', msg.data)
        self.send_pano()
        self.send_pov()

    def handle_pano_msg(self, msg):
        """
        Stores any panos that are broadcasted on /panoviewer/pano
        """
        self.pano_url = msg.data

    def handle_pov_msg(self, msg):
        self.pov = msg

def clamp(val, low, high):
    return min(max(val, low), high)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
