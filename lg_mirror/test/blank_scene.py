#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage

if __name__ == '__main__':
    scene_msg = GenericMessage()
    scene_msg.type = 'json'
    scene_msg.message = '{}'

    rospy.init_node('director_messager')
    scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=100)

    rospy.sleep(2)
    scene_pub.publish(scene_msg)
    rospy.sleep(2)
