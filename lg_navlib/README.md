lg\_navlib
----------

nav\_mode.py
============

Controller for inferring multi-touch to SpaceNav axis mapping from content changes.

Liquid Galaxy can include a number of applications, such as 3D globes, panoramic and flat image and video viewers, and so on.  Typically most of these applications are hidden in the background until a change in content demands their visibility.  The purpose of the nav\_mode script is inferring what type of content is currently "in the spotlight" such that other nodes can function accordingly.

The intention is that this information is used to map multi-touch events to SpaceNav inputs.  Because the multi-touch transformation occurs in a web application separate from this repo, the signal for which mode to use is distilled to a single topic.

A typical configuration is for globe or "top-down" viewers (Earth, Cesium, Maps) to be a separate mode from "eye-level" viewers (Street View, Unity, panoramic images and videos).  This informs the user interface to map multi-touch events to appropriate SpaceNav axes.
