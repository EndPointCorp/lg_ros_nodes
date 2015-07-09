import rospy

class PointcloudMaster:
    def spacenavTwist(self, msg):
        rospy.loginfo('Received twist msg: %s', msg)
