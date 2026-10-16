lg\_common
----------

Common software for Liquid Galaxy ROS nodes. Basic browser ros scripts
that provide adhoc or static browser activity + some helpers shared by
all LG ros nodes.

## Hardware requirements

* it's good to have decent CPU and accelarated graphics for Google
  Chrome to work smoothly

## Software requirements

* awesome window manager
* google chrome

#### Messages
- AdhocBrowser - contains all information needed to start and fully
  configure an application inside of a Google Chrome browser
- AdhocBrowsers - contains list of AdhocBrowser messages
- Ready - message emitted from within an AdhocBrowser instance (e.g.
  google-chrome-stable extension) that notifies 3rd party about the fact
that DOM inside the browser acquired a `loaded` state

## Scripts

### adhoc\_browser.py

Provides a browser pool for running and managing chrome browser instances with some content loaded into them.

#### Parameters

* `~viewport` [string] - name of the viewport to run at. This is a mandatory argument.
* `~extensions_root` [string] - absolute or relative path to directory with unpacked chrome extensions - defaults to `/opt/endpoint/chrome/extensions/`
* `~depend_on_rosbridge` [bool] - wait for rosbridge to answer before launching. Default: `true`
* `~hide_delay` [float] - seconds to wait after unhiding a scene's browsers before hiding the preloaded ones they replace. Default: `0.5`
* `~destroy_delay` [float] - further seconds to wait after hiding those old browsers before destroying them. Default: `2`

    ```json
    {
      "activity_config": {
        "preload": true,
          "google_chrome":{
            "extensions": [
            {
              "name": "test_extension1"
            },
            {
              "name": "test_extension2"
            }
            ]
          }
      }
    }
    ```

    thanks to this parameter - you may emit a message with above `activity_config` without absolute path to the extension

* `~rosbridge_secure` (boolean) and `~rosbridge_port` these two parameters are passed via get arguments to all created browsers, to provide rosbridge connection data for extensions and applications.

##### Subscribed Topics

* `/browser_service/<viewport>` [`lg_adhoc_browser/AdhocBrowsers`] - A list of browsers which should be opened.
* `/director/scene`

#### Messages

##### AdhocBrowsers

* Contains a list of browsers

##### AdhocBrowser

* string `id` - static ID of the browser - generated from its attributes
  to prevent restarts of identical browsers across scenes
* string `url` - URL that browser will be started up with
* string `scene_slug` - slug of a scene from USCS message that contained
  the asset
* bool `preload` - flag for turning
  [preloading](https://github.com/EndPointCorp/lg_ros_nodes/wiki/Unified-State-Control-System-API#preloading) on/off
* bool `custom_preload_event` - name of the custom event that will
  trigger preloading (defined
[here](https://github.com/EndPointCorp/lg_ros_nodes/tree/master/lg_common/src/lg_common/extensions/ros_window_ready#application-generated-message))
* string `user_agent` - user agent of the browser
* string `version` - browser version - defaults to 'stable' (available:
  `beta` and `unstable`)
* lg_common/WindowGeometry `geometry` - geometry of the window (defined
  below)
* lg_common/BrowserExtension[] `extensions` - list of extensions' names.
  Extension won't be loaded if it does not exist. By default extension
will be looked for in `lg_common/src/lg_common/extensions` and under
`/opt/endpoint/chrome/extensions` as a fallback.
* lg_common/BrowserCmdArg[] `command_line_args` - additional command
  line arguments provided and appended verbatim e.g. `--enable-nacl`

##### WindowGeometry

- int32 `x` - x offset of the position of window, relative to Xorg point
  0,0
- int32 `y` - y offset of the position of window, relative to Xorg point
- uint32 `width` - width of the window
- uint32 `height` - height of the window

### dev\_webserver.py

A static HTTP server for the ROS share path. The location of the ROS share path is the parent directory of the `lg_common` package, which depends on which ROS `setup.bash` was sourced.

If you ran `catkin_make` and sourced `devel/setup.bash`, it will serve the `src/` path of your Catkin workspace. This is good for development.

If you ran `catkin_make install` and sourced `install/setup.bash`, it will serve the `install/share` path of your Catkin workspace. This is good for automated testing, since it verifies that `CMakeLists.txt` is complete.

If you installed `lg_common` from a `.deb` package and sourced the default `/opt/ros/noetic/setup.bash`, it will serve the system `/opt/ros/noetic/share` path. This is good for production (for now).

#### Parameters

* `port` [int] - The port on which to serve files. Default: `8008`

#### Convention for webapps

If your ROS package has a static webapp you'd like to be served, put files in the package's `webapps/` and add a snippet to the bottom of `CMakeLists.txt`:

    install(
      DIRECTORY webapps/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
    )

For example, there is an `index.html` located in `lg_common/webapps/example/` which can be accessed at `http://localhost:8008/lg_common/webapps/example/index.html`.

### static\_browser.py

Puts an always-visible browser on the screen.

#### Parameters

* `~viewport` [string] - name of the viewport to run at. This is a mandatory argument.
* `~url` [string] - the browser will be pointed at this url.
* `~command_line_args` [string] - the browser will run with these additional args.
* `~force_device_scale_factor` [float] - override `devicePixelRatio` to this value.  Default: `1`
* `~extra_logging` [bool] - enabled additional logging for this instance.  Default: `false`
* `~user_agent` [string] - override use agent to this value.  The default is to spoof an iPad.
* `~kiosk` [bool] - run the browser in kiosk mode.  Default: `true`
* `~state` [ApplicationState] - set the browser to this state.  Default: `VISIBLE`
* `~extensions` [string] - list of extensions.
* `~depend_on_url` [bool] - wait for the url to be available before launching?  Default: `false`
* `~user_data_dir` [string] - Chrome profile directory for this browser. Default: `None`, so Chrome picks its own.
* `~debug_port` [int] - serve the Chrome remote debugging protocol on this port. Default: `None`, which leaves it off.

----------------------

### touchscreen.py

Puts the touchscreen web app on a viewport, pointed at the director and
rosbridge it should talk to.

#### Parameters

* `~url` [string] - point the browser here instead of assembling a url from the parameters below.
* `~url_base` [string] - base the assembled url on this. Default: `http://lg-head/ros_touchscreens/ts/`
* `~ts_name` [string] - touchscreen name, appended to `~url_base` to select which interface to load. Default: `default`
* `~director_host` [string] - director host embedded in the assembled url. Default: `42-a`
* `~director_port` [int] - director port embedded in the assembled url. Default: `8060`
* `~director_secure` [int] - use https for the director in the assembled url. Default: `0`
* `~rosbridge_host` [string] - rosbridge host embedded in the assembled url. Default: `localhost`
* `~depend_on_rosbridge` [bool] - wait for rosbridge to answer before launching. Default: `false`
* `~depend_on_director` [bool] - wait for the director to answer before launching. Default: `false`
* `~force_device_scale_factor` [float] - override `devicePixelRatio` to this value. Default: `1`
* `~debug_port` [int] - serve the Chrome remote debugging protocol on this port. Default: `None`, which leaves it off.

----------------------

### uscs\_service.py

Holds the state a display should return to, and republishes it on the
director topic when the system comes online, goes offline, or changes
activity.

#### Parameters

* `~director_topic` [string] - topic the service publishes scenes on. Default: `/director/scene`
* `~message_topic` [string] - topic the service publishes its own state messages on. Default: `/uscs/message`
* `~offline_topic` [string] - topic watched for the offline signal. Default: `/lg_offliner/offline`
* `~activity_topic` [string] - topic watched for the activity signal. Default: `/activity/active`
* `~depend_on_scene_repository` [bool] - wait for the scene urls below to be reachable before starting. Default: `true`

----------------------

### lg\_common module

#### lg\_common.ManagedApplication

Combined process and window manager.

#### lg\_common.ManagedBrowser

A `ManagedApplication` subclass for running a browser.

##### args

* `url` [string] - URL to open. Optional.
* `slug` [string] - A unique slug for this browser window. If none is provided, `ManagedBrowser` will attempt to use the node name.
* `kiosk` [bool] - If true, launch the browser in kiosk mode. Default: `true`
* `geometry` [WindowGeometry] - Where to put the window. Optional.
* `binary` [string] - Absolute path to the browser binary. Default: `/usr/bin/google-chrome`
* `remote_debugging_port` [int] - Specify a remote debugging port. If not provided, a port will be assigned.
* `command_line_args` [string] - A large string of command line arguments for chrome.
* `log_stderr` [bool] - Whether or not to log standard error (fills up quickly
  for debugging only) Default: `False`

All other keyword arguments are passed on directly to the command line.

##### Parameters

`ManagedBrowser` reads these itself, so they apply to any node that builds one.

* `~remove_default_args` [string] - strip Chrome's default argument list.
  Falls back to the global `/remove_default_args`. Compared as a lowercased
  string, so `"true"` enables it. Default: `"false"`

See `examples/browser.py` for an example implementation.

See `src/lg_common/managed_browser.py` for default args.

#### lg\_common.ManagedWindow

Window manager.

#### lg\_common.SceneListener

Runs a callback upon scene messages.
