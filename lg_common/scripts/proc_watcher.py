#!/usr/bin/env python

import rospy
from lg_common import ProcWatcher


def main():
    rospy.init_node('process_watcher')
    cpu_time = rospy.get_param('~cpu_time', None)
    cpu_usage = rospy.get_param('~cpu_usage', None)
    inactive_time = rospy.get_param('~inactive_time', 30)
    memory_usage = rospy.get_param('~memory_usage', None)
    diskspace_usage = rospy.get_param('~diskspace_usage', None)
    diskspace_dirs = rospy.get_param('~diskspace_dirs', [])
    interval = rospy.get_param('~interval', 5)

    if type(interval) != int and type(interval) != float:
        rospy.logerr('Invalid (non float/int) interval supplied, using 5')
        interval = 5

    if diskspace_dirs:
        diskspace_dirs = diskspace_dirs.split(',')
    else:
        diskspace_dirs = []

    node_name = rospy.get_param('~node_name', None)
    if node_name is None:
        rospy.logerr('There was no node name given, killing self')
        exit(1)

    if node_name[0] != '/':
        node_name = '/' + node_name

    s = 'making proc watcher with\n {node_name}, cpu_time={cpu_time}, inactive_time={inactive_time}, cpu_usage={cpu_usage}, memory_usage={memory_usage}, diskspace_usage={diskspace_usage}, diskspace_dirs={diskspace_dirs}'.format(
    node_name=node_name, cpu_time=cpu_time, inactive_time=inactive_time,
                    cpu_usage=cpu_usage, memory_usage=memory_usage,
                    diskspace_usage=diskspace_usage,
                    diskspace_dirs=diskspace_dirs
        )
    rospy.loginfo(s)
    p = ProcWatcher(node_name, cpu_time=cpu_time, inactive_time=inactive_time,
                    cpu_usage=cpu_usage, memory_usage=memory_usage,
                    diskspace_usage=diskspace_usage,
                    diskspace_dirs=diskspace_dirs)

    rospy.loginfo('using %s' % interval)
    rospy.Timer(rospy.Duration(interval), p.run)
    rospy.spin()

if __name__ == '__main__':
    main()
