#!/usr/bin/env python

import rospy
from lg_common.msg import GetScreenshot
from lg_common.msg import Screenshot



def main():
    rospy.init_node('lg_web_screenshot')
    binary = rospy.get_param('~binary', '/usr/bin/phantomjs')
    script = rospy.get_param('~script', 'screenshots.js')
    delay = rospy.get_param('~delay', 250)
    ua = rospy.get_param('~user_agent', DEFAULT_UA)

    screenshot_publisher = rospy.Publisher('/screenshot/screenshot',
                                           Screenshot,
                                           queue_size=20)

    node = SearchScreenshot(screenshot_publisher,
                            binary=binary,
                            script=script,
                            delay=delay)

    rospy.Subscriber('/screenshot/get',
                     GetScreenshot,
                     node.take_screenshot)


if __name__ == "__main__":
    main()
