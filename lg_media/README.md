lg\_media
---------

ROS software for managing ad hoc media service (mplayer application).

Spec on (lg_ros_nodes/issues/31 - media player - a service that
spins up audio/video players on demand.

### Nodes

#### browser\_launcher

Subscribes to media messages, and creates the browser pool with each media
message translated into a URL for a local videosync server to play.

##### Parameters

* `viewport` [string] - The viewport this media will be played on. Default:
  `center`
* `leader` [boolean] - "true" or "false" for whether or not this is the one
  leader for the current video. Default: `false`
* `ros\_port` [int] - the port rosbridge is running on \<ros\_host\>. Default:
  `9090`
* `ros\_host` [string] - the hostname rosbridge is running on. Default:
  `localhost`
* `videosync_url` [string] - the location of the videosync webapp. Default:
  `http://lg-head/lg_sv/webapps/videosync/index.html`
* `sync_rate` [int] - Maximum video sync rate in Hz. Default: `60`
* `frame_latency` [float] - Followers increment the leader's time by this fixed
  amount to compensate for frame delay. Default: `3 / 25`
* `ping_interval` [int] - Followers send a ping at this interval. Default:
  `1000`
* `hard_sync_diff` [float] - Seek to sync if time difference is greater than
  this amount. Default: `1.0`
* `min_playbackrate` [float] - Never set playbackRate below this. Default: `0.5`
* `max_playbackrate` [float] - Never set playbackRate above this. Default: `1.0`

##### Published Topics

* `/media_service/launch_browser` [`lg\_common/AdhocBrowsers`] - Publishes the
  current browsers in the browser pool.

##### Subscribed Topics

* `/media_service/launch_browser` [`lg\_common/AdhocBrowsers`] - The browser
  pool listens on this topic for adhoc browsers.

* `/media_service/<viewport>` [`lg\_media/AdhocMedias`] - Gets a list of medias
  that need to be translated into browsers.

#### browser\_player

This node listens on `/director/scene` and passes any media messages to
`/media\_service/<viewport>` for them to be launched by the `browser\_launcher`
##### Parameters

* `viewport` [string] - The viewport this media will be played on. Default:
  `center`

##### Published Topics

* `/media_service/<viewport>` [`lg\_media/AdhocMedias`] - The director bridge
  will publish medias on this topic.

##### Subscribed Topics

* `/director/scene` [`interactivespaces\_msgs/GenericMessage`] - Director
  messages on this topic will be translated into `AdhocMedias` thanks to the
  `DirectorMediaBridge`
