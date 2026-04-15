#!/usr/bin/env python3

import visionport.vpros as rospy
import json
from visionport.vpros.models.interactivespaces_msgs.msg import GenericMessage
from lg_videos_replay.player import VideoReplayPlayer

class VideosReplayNode:
    def __init__(self):
        self.recordings_path = rospy.get_param('~recordings_path', '/mnt//videos/tmp')
        self.recordings_log = rospy.get_param('~recordings_log', '/mnt//videos/recordings.jsonl')

        self.player = VideoReplayPlayer(self.recordings_log, self.recordings_path)

        rospy.Subscriber('/lg_videos_replay', GenericMessage, self.handle_message)
        rospy.loginfo(f"lg_videos_replay initialized. Watching log: {self.recordings_log}")

    def handle_message(self, msg):
        msg_type = msg.type
        try:
            payload = json.loads(msg.message) if msg.message else {}
        except json.JSONDecodeError:
            rospy.logerr(f"Failed to parse message payload: {msg.message}")
            payload = {}

        if msg_type in ("play", "start"):
            timestamp = payload.get("timestamp")
            if timestamp is None:
                rospy.logerr("Play/start command requires 'timestamp' in the message payload")
                return
            rospy.loginfo(f"Received {msg_type} command for timestamp {timestamp}")
            self.player.play(timestamp)

        elif msg_type == "stop":
            rospy.loginfo("Received stop command")
            self.player.stop()

        else:
            rospy.logwarn(f"Unknown command type: {msg_type}")

    def shutdown(self):
        self.player.stop()

if __name__ == '__main__':
    rospy.init_node('lg_videos_replay', anonymous=True)
    node = VideosReplayNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
