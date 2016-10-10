#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from lg_common.msg import BrowserURL
from lg_common.msg import AdhocBrowsers
from interactivespaces_msgs.msg import GenericMessage


def main():
    rospy.init_node('browser_current_url', anonymous=False)

    viewports = [ v.split('/')[-1] for v in rospy.get_param_names() if v.startswith('/viewport')]

    url_service = BrowserURLService(viewports)
    rospy.Subscriber('/browser/extension/current_url', BrowserURL, url_service.handle_url_message)


    for viewport in viewports:
        topic_name = '/browser_service/{}'.format(viewport)
        rospy.Subscriber(topic_name,
                         AdhocBrowsers,
                         lambda msg: url_service.handle_browsers(msg, viewport))

        rospy.Service('/browser/{}/current_url'.format(viewport),
                      BrowserURLService,
                      lambda: return url_service.get_browser(viewport))

    rospy.Service('/browser/current_url',
                  BrowserURLService,
                  url_service.get_all_browsers)

    rospy.spin()


if __name__ == '__main__':
    main()
