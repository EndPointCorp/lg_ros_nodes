lg_attract_loop
---------------

# Nodes

## attract_loop.py

Attract loop is a kind of screen saver played back on the Liquid Galaxy
if it was inactive for `lg_activity/activity_timeout` number of seconds.
Whenever above happens, lg_attract_loop will get the scenes and
presentations from DIRECTOR_API_URL (that proxies the requests to ROS
content management system where presentations are configured) and will
play them back.

### Parameters

* `~activity_topic` [string] - topic where activity messages are
  published
* `~director_scene_topic_name` [string] - ROS topic name where director
  scene messages should be publishe
* `~director_presentation_topic_name` [string] - ROS topic name where
  director presentation messages should be published

### Subscribed topics

* activity topic parametrized via `~activity_topic` ROS param. Most
  likely `/activity/active` [Bool]

### Published topics

* director scene topic - most likely `/director/scene`
* director presentation topic - most likely `/director/presentation`
