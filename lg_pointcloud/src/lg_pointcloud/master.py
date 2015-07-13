import rospy

class PointcloudMaster:
    def spacenavTwist(self, msg):
        if (msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            return 
        rospy.loginfo('Received twist msg: %s', msg)
