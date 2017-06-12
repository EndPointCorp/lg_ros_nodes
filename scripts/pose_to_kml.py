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

def parse(data):
    reader = csv.DictReader(data)
    for row in reader:
        # In all serious GIS lon is X and lat is Y,
        # but we messed it up, and print lon as Y.
        result = {
            'lat': float(row['field.pose.position.x']),
            'lon': float(row['field.pose.position.y']),
            'tilt': float(row['field.pose.orientation.x']),
            'head': float(row['field.pose.orientation.y'])
        }
        divisor = sin(radians(result['tilt']))
        if fabs(result['tilt']) < 1.0:
            result['range'] = float(row['field.pose.position.z'])
        else:
            result['range'] = float(row['field.pose.position.z']) / divisor
        return result

def format_kml(data):
    return KML_TEMPLATE.format(0,
                               lat=data['lat'],
                               lon=data['lon'],
                               head=data['head'],
                               tilt=data['tilt'],
                               range=data['range'])

if __name__ == "__main__":
    msg = parse(sys.stdin)
    print format_kml(msg)
