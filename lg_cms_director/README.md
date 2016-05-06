# Liquid Galaxy CMS director

Director provides a bridge (with state) between Chrome browser, that provides
the Liquid Galaxy touchscreen interface, and ROS stack. It emits
`interactivespaces_msg.msg.GenericMessage`.

Touchscreen uses the director that communicates with the non-opensource CMS `director_api` to get
presentation and scenes for the touchscreen.

Once the content is loaded into touchscreen interface, it communicates
with ROS via rosbridge.

See interactivespaces_msgs README.md for more information about driving the ROS nodes.

# Nodes

## director.py

### Parameters

* `director_api_url` [string]: URL to API of non-opensource CMS

### Published topics

* `/director/scene` - main topic where
  interactivespaces_msgs.msg.GenericMessage json messages are published.
This topic will contain the absolute adhoc data for driving all other
ros nodes. You don't have to be running director to be able to display
content and be using on Liquid Galaxy.
* `/director/presentation` - every scene that was published on
  `/director/scene` belongs to presentation that should be published on
this topic
* `/director/presentationgroup` - every presentation that was published on
  `/director/presentation` belongs to presentationgroup that should be published on
this topic

### Subscribed topics

Since director communication works both ways - director subscribes to
the same topics it publishes on.
