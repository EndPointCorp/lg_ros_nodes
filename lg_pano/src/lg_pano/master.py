import rospy

class PanoMaster:
    def spacenavTwist(self, msg):
        if (msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0):
            return 
        rospy.loginfo('Received twist msg: %s', msg)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
