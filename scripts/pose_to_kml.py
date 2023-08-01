#!/usr/bin/env python3

import sys
import csv
from math import sin, radians, fabs

KML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2"
    xmlns:kml="http://www.opengis.net/kml/2.2"
    xmlns:atom="http://www.w3.org/2005/Atom">
<Document>
    <name>KmlFile</name>
    <Placemark>
        <name>Placemark 1</name>
        <LookAt>
            <longitude>{lon}</longitude>
            <latitude>{lat}</latitude>
            <altitude>0</altitude>
            <heading>{head}</heading>
            <tilt>{tilt}</tilt>
            <range>{range}</range>
            <gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>
        </LookAt>
        <Point>
            <gx:drawOrder>1</gx:drawOrder>
            <coordinates>{lon},{lat},0</coordinates>
        </Point>
    </Placemark>
</Document>
</kml>
"""

def parseCSV(data):
    reader = csv.DictReader(data)
    for row in reader:
        # In all GIS lon is X and lat is Y,
        # but we messed it up, and print lon as Y.
        return calc(
            row['field.pose.position.y'],
            row['field.pose.position.x'],
            row['field.pose.position.z'],
            row['field.pose.orientation.x'],
            row['field.pose.orientation.y']
        )

def calc(x, y, z, heading, tilt):
    result = {
        'lon': float(x),
        'lat': float(y),
        'tilt': float(tilt),
        'head': float(heading)
    }
    divisor = sin(radians(result['tilt']))
    if fabs(result['tilt']) < 1.0:
        result['range'] = float(z)
    else:
        result['range'] = float(z) / divisor
    return result

def parseCLI():
    return calc(
        sys.argv[1],
        sys.argv[2],
        sys.argv[3],
        sys.argv[4],
        sys.argv[5]
    )

def format_kml(data):
    return KML_TEMPLATE.format(0,
                               lat=data['lat'],
                               lon=data['lon'],
                               head=data['head'],
                               tilt=data['tilt'],
                               range=data['range'])

if __name__ == "__main__":
    msg = parse(sys.stdin)
    print(format_kml(msg))
