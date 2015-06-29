lg\_common
----------

Common software for Liquid Galaxy ROS nodes.

### Scripts

#### dev\_webserver.py

A static HTTP server for the ROS share path. The location of the ROS share path is the parent directory of the `lg_common` package, which depends on which ROS `setup.bash` was sourced.

If you ran `catkin_make` and sourced `devel/setup.bash`, it will serve the `src/` path of your Catkin workspace. This is good for development.

If you ran `catkin_make install` and sourced `install/setup.bash`, it will serve the `install/share` path of your Catkin workspace. This is good for automated testing, since it verifies that `CMakeLists.txt` is complete.

If you installed `lg_common` from a `.deb` package and sourced the default `/opt/ros/indigo/setup.bash`, it will serve the system `/opt/ros/indigo/share` path. This is good for production (for now).

##### Parameters

* `port` [int] - The port on which to serve files. Default: `8008`
* `cors` [bool] - If true, add `Access-Control-Allow-Origin: *` to response headers. Default: `true`

##### Convention for webapps

If your ROS package has a static webapp you'd like to be served, put files in the package's `webapps/` and add a snippet to the bottom of `CMakeLists.txt`:

    install(
      DIRECTORY webapps/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
    )

For example, there is an `index.html` located in `lg_common/webapps/example/` which can be accessed at `http://localhost:8008/lg_common/webapps/example/index.html`.

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

All other keyword arguments are passed on directly to the command line.

See `examples/browser.py` for an example implementation.

See `src/lg_common/managed_browser.py` for default args.

#### lg\_common.ManagedWindow

Window manager.

#### lg\_common.SceneListener

Runs a callback upon scene messages.

