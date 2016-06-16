#!/usr/bin/env python
import rospy
import json
import os
import sys
import socket
import commands


class DebuggingDirector:
    def __init__(self):
        print sys.argv
        try:
            self.director_message_file = sys.argv[1]
            print "Going to use file: %s" % self.director_message_file
        except Exception:
            print "you must provide path to director message json file and remote host (optional)"
            exit(1)
        try:
            self.remote_host = sys.argv[2]
            print "Going to copy script with file to remote host: %s" % self.remote_host
        except Exception:
            self.remote_host = None

        if self.remote_host:
            print "-------- executing local commands --------"
            self._prepare()
        else:
            print "-------- executing remote commands on %s --------" % self.remote_host
            self._emit_message()

    def _prepare(self):
        print "Going to copy %s to %s:/tmp" % (self.director_message_file, self.remote_host)
        command = 'scp %s %s:/tmp' % (self.director_message_file, self.remote_host)
        print "CMD: %s" % command
        status, output = commands.getstatusoutput(command)
        if status != 0:
            print "JSON copy failed - exiting"
            exit(1)
        else:
            print "JSON copied"
        command = 'scp %s %s:/tmp' % (sys.argv[0], self.remote_host)
        print "CMD: %s" % command
        status, output = commands.getstatusoutput(command)
        if status != 0:
            print "send_director.py file copy failed - exiting"
            exit(1)
        else:
            print "Copied send_director.py"

        print "Executing CMD remotely:"
        command = "ssh -t -o ControlMaster=no %s '. /opt/ros/indigo/setup.bash; . ~/.bashrc; . /home/lg/catkin_ws/devel/setup.bash; . /etc/profile; python /tmp/send_director.py /tmp/%s'" % (self.remote_host, os.path.basename(self.director_message_file))
        print "CMD: %s" % command
        status, output = commands.getstatusoutput(command)
        if status != 0:
            print "remote execution failed - exiting"
            exit(1)
        else:
            print "Executed ssh command successfully"

    def _emit_message(self):
        print "Opening %s for publishing" % self.director_message_file
        with open(self.director_message_file, 'r') as uscs_msg:
            uscs_msg = uscs_msg.read()
        from interactivespaces_msgs.msg import GenericMessage
        self.msg = GenericMessage()
        self.msg.type = 'json'
        self.msg.message = uscs_msg

    def _init_ros_node(self):
        rospy.init_node('debugging_site', anonymous=True)
        rospy.sleep(1)

    def send(self):
        if self.remote_host:
            print "Remote host: %s" % self.remote_host
            print "Launched with params: %s" % sys.argv
            print "Just copying stuff to %s" % self.remote_host
        else:
            print "Emitting message"
            self._init_ros_node()
            from interactivespaces_msgs.msg import GenericMessage
            rospy.Publisher('/director/scene', GenericMessage, queue_size=10).publish(self.msg)
            print self.msg
            rospy.sleep(1)

if __name__ == "__main__":
    dd = DebuggingDirector()
    dd.send()
