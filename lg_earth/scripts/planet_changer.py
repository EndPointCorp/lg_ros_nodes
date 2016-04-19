#!/usr/bin/env python

import json
import rospy
from std_msgs.msg import String
from lg_common.helpers import get_params
from interactivespaces_msgs.msg import GenericMessage


QUERY_PLANET_EARTH = String('earth')
QUERY_PLANET_MOON = String('moon')
QUERY_PLANET_MARS = String('mars')


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


if __name__ == '__main__':
    rospy.init_node('earth_planet_changer')

    moon_presentations = str(
        get_params('~moon_presentations', 'Moon')
    ).split(';')
    mars_presentations = str(
        get_params('~mars_presentations', 'Mars')
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

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
