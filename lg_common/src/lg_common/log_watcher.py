#!/usr/bin/env python

from thread import start_new_thread
from time import sleep
import rospy

def MockCB(*args, **kwargs):
    pass

class LogWatcher(object):
    def __init__(self, logfile, alert_strings, cb=MockCB):
        self.alert_strings = alert_strings
        self.cb = cb
        self.logfile = logfile

    def run(self):
        start_new_thread(self.watch, ())

    def watch(self):
        try:
            f = open(self.logfile, 'r')
        except IOError:
            rospy.logwarn('No file (%s) found, retrying in 0.5 seconds' % self.logfile)
            sleep(0.5)
            self.watch()
            return
        f.read()  # skip to the bottom of the file to ignore old errors
        while True:
            line = f.readline()
            for alert in self.alert_strings:
                if alert and alert in line:
                    self.cb()

    @staticmethod
    def get_ros_process(node_name):
        for process in psutil.process_iter():
            if '__name:=%s' % node_name in process.cmdline:
                return process

    @staticmethod
    def get_log_file(node_name):
        log_prefix = '__log:='
        p = LogWatcher.get_ros_process(node_name)
        for arg in p:
            if arg.startswith(log_prefix):
                return arg[len(log_prefix):]
