import rospy

from geometry_msgs.msg import PoseStamped, Pose
from lg_common.msg import ApplicationState

class TouchscreenRelay(object):
    def __init__(self, publisher):
        self.earth_active = True
        self.publisher = publisher

    def handle_state_earth(self, msg):
        self.earth_active = msg.state != ApplicationState.VISIBLE

    def handle_earth_pose(self, msg):
        if self.earth_active:
            self.publisher.publish(msg)

    def handle_cesium_pose(self, msg):
        if not self.earth_active:
            stamped = PoseStamped()
            stamped.pose = msg
            x = msg.position.y
            y = msg.position.x
            stamped.pose.position.x = x
            stamped.pose.position.y = y
            self.publisher.publish(stamped)


if __name__ == "__main__":
    rospy.init_node('touchscree_pose_relay')

    touchscreen_pose_publisher = rospy.Publisher(
        '/touchscreen/free_flight_pose',
        PoseStamped,
        latch=True,
        queue_size=1)

    relay = TouchscreenRelay(touchscreen_pose_publisher)

    rospy.Subscriber('/earth/state', ApplicationState, relay.handle_state_earth)
    rospy.Subscriber('/earth/pose', PoseStamped, relay.handle_earth_pose)
    rospy.Subscriber('/cesium/free_flight_pose', Pose, relay.handle_cesium_pose)

    rospy.spin()
