#!/usr/bin/env python

import rospy
from lg_screenshot.msg import GetScreenshot
from lg_screenshot.msg import Screenshot
from lg_screenshot import WebScreenshot


DEFAULT_UA = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_12_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/54.0.2840.71 Safari/537.36"


def main():
    rospy.init_node('lg_web_screenshot')
    binary = rospy.get_param('~binary', '/usr/bin/phantomjs')
    script = rospy.get_param('~script', 'screenshots.js')
    delay = rospy.get_param('~delay', 250)
    user_agent = rospy.get_param('~user_agent', DEFAULT_UA)

    screenshot_publisher = rospy.Publisher('/screenshot/screenshot',
                                           Screenshot,
                                           queue_size=20)

    node = WebScreenshot(screenshot_publisher,
                         binary=binary,
                         script=script,
                         user_agent=user_agent,
                         delay=delay)

    rospy.Subscriber('/screenshot/get',
                     GetScreenshot,
                     node.take_screenshot)

    rospy.spin()

if __name__ == "__main__":
    main()
