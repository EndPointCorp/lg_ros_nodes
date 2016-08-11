# interactivespaces_msgs.msg.GenericMessage

```
type string
message string
```

Message type used by the lg_cms_director to drive other Liquid Galaxy
ros nodes to display data using non-opensource CMS.

This type is very generic and contains stringified (or serialized) json that should be always deserialized on
the Subscriber side. Below you'll find a description of how to show your
content on the touchscreen without running director and non-opensource
CMS developed by [End Point Corporation](http://endpoint.com)

Example on how to load an asset available under this link -
http://lg-head/lg/assets/kml/population.kml, into 3 google earth instances: left,
center and right:

```
type: json
message: {
  "description": "",
  "duration": 20,
  "name": "KML test",
  "resource_uri": "/director_api/scene/kml-test/",
  "slug": "kml-test",
  "windows": [
    {
      "activity": "earth",
      "assets": [
        "http://lg-head/lg/assets/kml/population.kml"
      ],
      "height": 1920,
      "presentation_viewport": "left",
      "width": 1080,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "earth",
      "assets": [
        "http://lg-head/lg/assets/kml/population.kml"
      ],
      "height": 1920,
      "presentation_viewport": "center",
      "width": 1080,
      "x_coord": 0,
      "y_coord": 0
    },
    {
      "activity": "earth",
      "assets": [
        "http://lg-head/lg/assets/kml/population.kml"
      ],
      "height": 1920,
      "presentation_viewport": "right",
      "width": 1080,
      "x_coord": 0,
      "y_coord": 0
    },
  ]
}
---
```

Single interactivespaces message contains full adhoc state of the Liquid
Galaxy.

As you see, every message contains top level attributes:

```json
  "description": "",
  "duration": 20,
  "name": "KML test",
  "resource_uri": "/director_api/scene/kml-test/",
  "slug": "kml-test",
```

and a list of windows that's picked up by specific instances of
applications (e.g. earth, streetview, mplayer) which will load assets
that are placed in the `assets` attribute of the single `window`.

By design - all applications instances (e.g. streetview right,
streetview center, streetview left) listen on assets addressed only to
them on specific `activity` name and `presentation_viewport`.
