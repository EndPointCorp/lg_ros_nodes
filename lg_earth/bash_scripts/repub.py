import rospy
import sys
import json
from interactivespaces_msgs.msg import GenericMessage
from lg_common.srv import USCSMessage, USCSMessageResponse, InitialUSCS, InitialUSCSResponse


try:
    rospy.init_node('director_messager')
    rospy.sleep(1)
    service = rospy.ServiceProxy('/uscs/message', USCSMessage)
    current_scene = service()
    clear_scene = stop_scene = " { \"description\": \"STOP the presentation\", \"duration\": 0, \"name\": \"stop the presentations\", \"resource_uri\": \"/director_api/scene/stop-the-presentations/\", \"slug\": \"stop-the-presentations\", \"windows\": [] } "
    clear_scene = json.loads(clear_scene)
    clear_msg = GenericMessage()
    clear_msg.type = 'json'
    clear_msg.message = json.dumps(clear_scene)
    rospy.Publisher('/director/scene', GenericMessage, queue_size=10, latch=True).publish(clear_msg)
    rospy.sleep(1)
    current_scene = str(current_scene)
    current_scene = current_scene.strip('type: json')
    current_scene = current_scene.strip('\n')
    current_scene = current_scene.strip('message:')
    current_scene = json.loads(current_scene)
    DIRECTOR_MESSAGE = json.dumps(current_scene)
    msg = GenericMessage()
    msg.type = 'json'
    msg.message = DIRECTOR_MESSAGE
    rospy.Publisher('/director/scene', GenericMessage, queue_size=10, latch=True).publish(msg)
    rospy.sleep(1)
except KeyboardInterrupt:
    print "Exiting cleanly"
    rospy.signal_shutdown("Ctrl+c used on send_director.py")
