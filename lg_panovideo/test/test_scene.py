#!/usr/bin/env python
import rospy
import sys
from interactivespaces_msgs.msg import GenericMessage

if len(sys.argv) > 1:
    video_url = sys.argv[0]
else:
    video_url = 'http://localhost/rangers.mp4'


DIRECTOR_MESSAGE = """
{
  "description": "bogus",
  "duration": 0,
  "name": "test whatever",
  "resource_uri": "bogus",
  "slug": "test message",
  "windows": [
    {
      "activity": "panovideo",
      "assets": [
        "%s"
      ],
      "activity_config": {
        "projection": "equirectangular",
        "expandCoef": 1.025,
        "loop": true
      },
      "presentation_viewport": "left",
      "width": 800,
      "height": 600,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "panovideo",
      "assets": [
        "%s"
      ],
      "activity_config": {
        "projection": "equirectangular",
        "expandCoef": 1.025,
        "loop": true
      },
      "presentation_viewport": "right",
      "width": 800,
      "height": 600,
      "x_coord": 800,
      "y_coord": 0
    }
  ]
}
""" % (video_url, video_url)

scene_msg = GenericMessage()
scene_msg.type = 'json'
scene_msg.message = DIRECTOR_MESSAGE

rospy.init_node('sv_messager')
scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=100)

rospy.sleep(2)
scene_pub.publish(scene_msg)
rospy.sleep(2)
