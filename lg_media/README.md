lg\_media
---------

ROS software for managing ad hoc media service (mplayer application).

Spec on (lg_ros_nodes/issues/31 - media player - a service that
spins up audio/video players on demand.

## Hardawre

* accelarated graphics

## Software

* awesome window manager for window positioning

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
* `max_playbackrate` [float] - Never set playbackRate above this. Default: `1.5`
* `autoplay` [bool] - Start playback without waiting for a user gesture. Default: `false`
* `show_controls` [bool] - Show the player's transport controls. Default: `false`

##### Published Topics

* `/media_service/launch_browser` [`lg\_common/AdhocBrowsers`] - Publishes the
  current browsers in the browser pool.

##### Subscribed Topics

* `/media_service/launch_browser` [`lg\_common/AdhocBrowsers`] - The browser
  pool listens on this topic for adhoc browsers.

* `/media_service/<viewport>` [`lg\_media/AdhocMedias`] - Gets a list of medias
  that need to be translated into browsers.

#### image\_viewer

Shows a scene's images on its viewports.

##### Parameters

* `~viewports` [string] - Comma-separated viewports this node draws on. Default: `''`
* `~save_dir` [string] - Directory under `/tmp` that fetched images are cached in. Default: `images`

#### image\_checker

Watches the images a scene asked for and reports the ones that never appeared.

##### Parameters

* `~viewports` [string] - Comma-separated viewports to watch. Default: `''`
* `~timeout_length` [int] - Seconds an image may take to appear before it is reported. Default: `8`

#### media\_launcher

Turns director scenes into the media messages the players consume.

##### Parameters

* `~viewports` [string] - Comma-separated viewports to serve. Read with no default, so it must be set.

#### Media pools

Both the mplayer and gstreamer pools read these when building a player
command line.

##### Parameters

* `~application_path` [string] - Player binary to run. Default: `mplayer` or `gst_video_sync`, depending on the pool.
* `~application_flags` [string] - Arguments passed to it. Defaults to the flag set each pool ships.

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

# TODO

- make mplayer director pool eval the URL to prevent it from opening URLs with spaces:
  e.g. this won't work: "http://lg-head/cms/1-5 Shokasonjuku Academy.avi'
