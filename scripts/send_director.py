import rospy
import sys
import json
from interactivespaces_msgs.msg import GenericMessage


"""
This script may be used to send director message.
You just need to supply a json file with director message
in it.
"""

if len(sys.argv) <= 1:
    print "Sorry - you need to supply path to json file for emission"
    print "e.g. ./script.py <path_to_json>"
    sys.exit(1)


try:
    json_file = open(sys.argv[1], 'r')
except IOError:
    print "Could not open file"

try:
    message = json_file.read()
except IOError:
    print "Could not read file"

try:
    message = json.loads(message)
    DIRECTOR_MESSAGE = json.dumps(message)
except ValueError:
    print "Could not parse json file"


msg = GenericMessage()
msg.type = 'json'
msg.message = DIRECTOR_MESSAGE

try:
    rospy.init_node('director_messager')
    rospy.sleep(1)
    rospy.Publisher('/director/scene', GenericMessage, queue_size=10, latch=True).publish(msg)
    rospy.sleep(1)
except KeyboardInterrupt:
    print "Exiting cleanly"
    rospy.signal_shutdown("Ctrl+c used on send_director.py")
