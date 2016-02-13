#!/usr/bin/env python

from lg_chrome_control import PageMonitor
from appctl_support import ModeHandler
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

from appctl_support.controller import BaseController
from threading import Lock
class TimerController(BaseController):
    def __init__(self, callback, second_duration=10):
        self.callback = callback
        self.duration = rospy.Duration(second_duration)
        self.lock = Lock()
        self.timer = None

    def start(self, *args, **kwargs):
        with self.lock:
            if self.timer is None or not self.timer.isAlive():
                self.timer = rospy.Timer(self.duration, self.callback)

    def stop(self, *args, **kwargs):
        with self.lock:
            if self.timer:
                self.timer.shutdown()
                self.timer = None

if __name__ == '__main__':
    main()
