#!/usr/bin/env python3
"""
ROS node to monitor DCV activity JSON and publish updates when interaction times change.
Publishes on topic /dcv_viewer/activity with message containing the activity dict and allowed_time param.
"""
import rospy
import json
import time
import os
from std_msgs.msg import String

class DcvActivityPublisher:
    def __init__(self):
        rospy.init_node('dcv_activity_publisher')
        self.pub = rospy.Publisher('/dcv_viewer/activity', String, queue_size=10)

        self.allowed_time = rospy.get_param('~allowed_time', 300)
        self.connections_file = rospy.get_param('~connections_file', '/mnt/videos/dcv_activity.json')
        self.poll_interval = rospy.get_param('~poll_interval', 1.0)  # seconds
        self.previous_data = {}

    def read_connections(self):
        """ Read the JSON file and return a dict of {id: last-interaction-time}"""
        try:
            with open(self.connections_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logwarn(f"Failed to read connections file: {e}")
            return {}

    def run(self):
        rate = rospy.Rate(1.0 / self.poll_interval) if self.poll_interval > 0 else None
        while not rospy.is_shutdown():
            current = self.read_connections()
            if current != self.previous_data:
                # Compose message payload
                payload = {
                    'allowed_time': self.allowed_time,
                    'activity': current
                }
                msg = String()
                msg.data = json.dumps(payload)
                self.pub.publish(msg)
                rospy.loginfo(f"Published updated DCV activity: {current}")
                self.previous_data = current

            # Sleep
            if rate:
                rate.sleep()
            else:
                time.sleep(self.poll_interval)

if __name__ == '__main__':
    try:
        node = DcvActivityPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass

