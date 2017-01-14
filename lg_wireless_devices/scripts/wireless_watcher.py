#!/usr/bin/env python
# THIS FILE IS MANAGED BY CHEF AND THIS IS JUST A PLACHOLDER

import os
import rospy

from lg_wireless_devices.common import get_config, get_and_place_file
from lg_wireless_devices import WirelessWatcher
from command_handler.msg import Command
from lg_common.helpers import run_with_influx_exception_handler


def main():
    rospy.init_node('wireless_watcher')

    rules_destination = '/etc/udev/rules.d/99-remote.rules'
    udev_location = rospy.get_param('~udev_location', 'http://lg-head/lg/external_devices/99-remote.rules')
    get_and_place_file(udev_location, '/tmp/99-remote.rules')
    os.system('sudo cp /tmp/99-remote.rules %s;sudo service udev restart; sudo udevadm trigger' % rules_destination)
    rospy.sleep(2)

    config_location = rospy.get_param('~config_location', 'http://lg-head/lg/external_devices/wireless_watcher.conf')
    config_destination = '/home/lg/etc/wireless_watcher.conf'
    get_and_place_file(config_location, config_destination)

    config = get_config(config_destination)
    command_handler = rospy.Publisher('/command', Command, queue_size=10)
    WirelessWatcher(config, command_handler)

if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
