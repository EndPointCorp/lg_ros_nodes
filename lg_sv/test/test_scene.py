#!/usr/bin/env python3
import rospy
from interactivespaces_msgs.msg import GenericMessage

P_SANDIEGO = '1ZPwYtCwgW6bu7gi7n3B4Q'
P_TEST = 'RJd2HuqmShMAAAQfCa3ulg'
P_PHOTOSPHERE = 'F:-gVtvWrACv2k/Vnh0Vg8Z8YI/AAAAAAABLWA/a-AT4Wb8MD8'
PANOID = P_SANDIEGO
DIRECTOR_MESSAGE = """
{
  "description": "bogus",
  "duration": 0,
  "name": "test whatever",
  "resource_uri": "bogus",
  "slug": "test message",
  "windows": [
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "center",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "right_one",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "right_two",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "right_three",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "left_one",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "left_two",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "streetview",
      "assets": [
        {
          "panoid": "%s"
        }
      ],
      "height": 1080,
      "presentation_viewport": "left_three",
      "width": 1920,
      "x_coord": 0,
      "y_coord": 0
    }
  ]
}
""" % (PANOID, PANOID, PANOID, PANOID, PANOID, PANOID, PANOID)

scene_msg = GenericMessage()
scene_msg.type = 'json'
scene_msg.message = DIRECTOR_MESSAGE

rospy.init_node('sv_messager')
scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=100)

rospy.sleep(2)
scene_pub.publish(scene_msg)
rospy.sleep(2)
