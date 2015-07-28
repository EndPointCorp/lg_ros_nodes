import rospy

from lg_common.msg import ApplicationState
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class PanoMaster:
    def __init__(self, pov, pano):
        self.povPublisher  = pov
        self.panoPublisher = pano
        self.pov = Vector3(0, 0, 0)
        self.panoUrl = String("")
        self.state = False

    def spacenavTwist(self, msg):
        """
        Changes the point of view based on incoming message

        ignores if state is False
        """
        if not self.state:
            return
        if (msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            return 
        self.pov.x += msg.angular.x
        self.pov.y += msg.angular.y
        self.pov.z += msg.angular.z
        self.sendPov()

    def sendPano(self):
        self.panoPublisher.publish(self.panoUrl)

    def sendPov(self):
        self.povPublisher.publish(self.pov)

    def handle_state(self, app_state):
        """
        Sets state to true, if visible
        """
        self.state = (app_state.state == ApplicationState.VISIBLE)

    def initApp(self, msg):
        # A new instance has connected; send it the current pano and pov
        # It doesn't really matter what the new instance sends; the presence of
        # any message tells us to send these messages.
        rospy.loginfo('Received init msg: %s', msg.data)
        self.sendPano()
        self.sendPov()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
