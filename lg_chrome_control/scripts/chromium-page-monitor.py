#!/usr/bin/env python

from lg_chrome_control import PageMonitor
from appctl_support import ModeHandler, TimerController
import rospy
import os

def grab_var_from_file(var, f='/home/lg/etc/shell.conf', default=''):
    import subprocess

    command = ['bash', '-c', 'source %s && echo $%s' % (f, var)]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    ret = proc.stdout.read().strip()
    proc.communicate()
    if not ret:
        ret = default
    return ret


def main():
    rospy.init_node('chromium_page_monitor')
    allowed_pages = rospy.get_param('~allowed_pages', '').split(';')
    bad_pages = rospy.get_param('~bad_pages', '').split(';')
    cmd_app = rospy.get_param('~cmd_app', 'gui')
    cmd_bin = rospy.get_param('~browser_bin',
                              grab_var_from_file('BROWSER_BIN'))
    port = rospy.get_param('~debug_port',
                           grab_var_from_file('BROWSER_DEBUG_PORT'))
    verbose = rospy.get_param('~verbose', False)
    modes = rospy.get_param('~modes', '').split(',')

    if allowed_pages == ['']:
        allowed_pages = []
    if bad_pages == ['']:
        bad_pages = []

    monitor = PageMonitor(cmd_app=cmd_app, cmd_bin=cmd_bin,
                          allowed_pages=allowed_pages, bad_pages=bad_pages,
                          port=port, verbose=verbose)

    tc = TimerController(monitor.monitor, 1)
    mh = ModeHandler(modes, tc)
    mh.begin_handling_modes()
    rospy.spin()

if __name__ == '__main__':
    main()
