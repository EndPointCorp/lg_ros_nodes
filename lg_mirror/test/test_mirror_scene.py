#!/usr/bin/env python
import rospy
from interactivespaces_msgs.msg import GenericMessage
DIRECTOR_MESSAGE = """
{
  "description": "bogus",
  "duration": 0,
  "name": "test whatever",
  "resource_uri": "bogus",
  "slug": "test message",
  "windows": [
    {
      "activity": "mirror",
      "activity_config": {
        "viewport": "center"
      },
      "assets": [
      ],
      "width": 450,
      "height": 800,
      "presentation_viewport": "left_one",
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "mirror",
      "activity_config": {
        "viewport": "center"
      },
      "assets": [
      ],
      "width": 450,
      "height": 800,
      "presentation_viewport": "right_one",
      "x_coord": 0,
      "y_coord": 0
    }
  ]
}
"""

if __name__ == '__main__':
    scene_msg = GenericMessage()
    scene_msg.type = 'json'
    scene_msg.message = DIRECTOR_MESSAGE

    rospy.init_node('director_messager')
    scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=100)

    rospy.sleep(2)
    scene_pub.publish(scene_msg)
    rospy.sleep(2)
