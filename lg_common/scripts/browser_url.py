#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from lg_common.msg import BrowserURL


def main():
    rospy.init_node('browser_current_url', anonymous=False)

    service = BrowserURLService()

    rospy.Service('/browser/current_url', BrowserURLService, service.get_browsers_urls)

    rospy.spin()


if __name__ == '__main__':
    main()
