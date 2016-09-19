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
      },
      "assets": [
        "viewport://center"
      ],
      "height": 480,
      "presentation_viewport": "left",
      "width": 640,
      "x_coord": 0,
      "y_coord": 0
    }
  ]
}
"""

scene_msg = GenericMessage()
scene_msg.type = 'json'
scene_msg.message = DIRECTOR_MESSAGE

rospy.init_node('director_messager')
scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=100)

rospy.sleep(1)
scene_pub.publish(scene_msg)
rospy.sleep(1)
