#!/usr/bin/env python3

import json
import rospy
from std_msgs.msg import String
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import run_with_influx_exception_handler


QUERY_PLANET_EARTH = String('earth')
QUERY_PLANET_MOON = String('moon')
QUERY_PLANET_MARS = String('mars')
NODE_NAME = 'planet_changer'


class PlanetChanger(object):
    def __init__(self, moon_presentations, mars_presentations, planet_pub):
        self.moon_presentations = moon_presentations
        self.mars_presentations = mars_presentations
        self.planet_pub = planet_pub

    def handle_presentation_msg(self, _msg):
        assert _msg.type == 'json'
        presentation = json.loads(_msg.message)
        pname = presentation['name']
        if pname in self.moon_presentations:
            self.planet_pub.publish(QUERY_PLANET_MOON)
        elif pname in self.mars_presentations:
            self.planet_pub.publish(QUERY_PLANET_MARS)
        else:
            self.planet_pub.publish(QUERY_PLANET_EARTH)


def main():
    rospy.init_node(NODE_NAME)

    moon_presentations = str(
        rospy.get_param('~moon_presentations', 'Moon')
    ).split(';')
    mars_presentations = str(
        rospy.get_param('~mars_presentations', 'Mars')
    ).split(';')

    planet_pub = rospy.Publisher('/earth/query/planet', String, queue_size=10)

    planet_changer = PlanetChanger(
        moon_presentations,
        mars_presentations,
        planet_pub
    )

    rospy.Subscriber(
        '/director/presentation',
        GenericMessage,
        planet_changer.handle_presentation_msg
    )

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
