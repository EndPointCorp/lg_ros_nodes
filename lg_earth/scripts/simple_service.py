#!/usr/bin/env python
import rospy
from lg_earth import OurState

def main():
    rospy.init_node('kml_service', anonymous=True)

    topic = rospy.get_param('~director_topic', '/director/scene')
    service_channel = rospy.get_param('~service_channel', 'kmlsync/state')
    s = OurState(topic, service_channel)

    rospy.spin()

if __name__ == '__main__':
    main()
