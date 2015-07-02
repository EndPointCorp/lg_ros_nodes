#!/usr/bin/env python
import rospy
from lg_earth import OurState

def main():
    rospy.init_node('kml_service', anonymous=True)

    s = OurState('/director/scene')

    rospy.spin()

if __name__ == '__main__':
    main()
