#!/usr/bin/env python3

import argparse
import json
import sys

import visionport.vpros as rospy
from visionport.vpros.models.interactivespaces_msgs.msg import GenericMessage


def parse_args():
    parser = argparse.ArgumentParser(
        description='Publish a start command to /lg_videos_replay for a given Unix timestamp.'
    )
    parser.add_argument(
        'timestamp',
        type=float,
        help='Unix timestamp to start replay from, for example 1737392750.5',
    )
    parser.add_argument(
        '--topic',
        default='/lg_videos_replay',
        help='Topic to publish to. Default: /lg_videos_replay',
    )
    return parser.parse_args()


def main():
    args = parse_args()

    rospy.init_node('lg_videos_play', anonymous=True)
    publisher = rospy.Publisher(args.topic, GenericMessage, queue_size=1, latch=True)

    msg = GenericMessage()
    msg.type = 'start'
    msg.message = json.dumps({'timestamp': args.timestamp})

    # Give ROS a brief moment to register the publisher before sending.
    rospy.sleep(0.5)
    publisher.publish(msg)
    rospy.loginfo(f'Published start command to {args.topic} for timestamp {args.timestamp}')
    rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(1)
