lg\_common
----------

Common software for Liquid Galaxy ROS nodes. Basic browser ros scripts
that provide adhoc or static browser activity + some helpers shared by
all LG ros nodes.

## Hardware requirements

* it's good to have decent CPU and accelarated graphics for Google
  Chrome to work smoothly

## Software requirements

* google-chrome available in PATH `~browser_binary`
* awesome window manager

## Scripts

### adhoc\_browser.py

ROS software for running and interfacing with the Chrome browser.

Listens to on a ROS topic, and creates, removes, and updates the browser windows.

Each ROS message needs to contain a list of all the browser windows to show. The browsers are distinguished using the `id` field.

The node manages internal list of opened browsers.

The list of browsers in the message is compared with the list of currently opened browsers, and then:

* if there is a browser, which `id` is not on the list, the browser is removed
* if there is not a browser for an `id`, then the browser is created
* if there already is a browser with the `id`, then the url and geometry are updated **[TODO: NOT IMPLEMENTED YET]**

#### Parameters

* `~viewport` [string] - name of the viewport to run at. This is a mandatory argument.
* `~browser_binary` [string] - absolute or relative path to browser binary

##### Subscribed Topics

* `/browser_service/<viewport>` [`lg_adhoc_browser/AdhocBrowsers`] - A list of browsers which should be opened.

#### Messages

##### AdhocBrowsers

* `browsers` - [`AdhocBrowser[]`]

##### AdhocBrowser

* `id` - [`string`] - Browser id.
* `geometry` - [`lg_common/WindowGeometry`] - Geometry of the browser window.
* `url` - [`string'] - Url to be loaded into the browser.

#### Adhoc browser example

The below examples assume that the node is configured like this:

```html
    <node name="image_browser" pkg="lg_adhoc_browser" type="client">
        <param name="viewport" value="superone" />
    </node>
```

* After a fresh start, there should be no browsers.

* This should show one window with the `endpoint.com` website:

```
rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}]"
```

* This should show another window, with the `google.com` website:

```
rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}, {id: '42x', geometry: {x: 600, y: 600, width: 300, height: 400}, url: 'http://google.com'}]"
```

* This should leave just the `endpoint.com` window shown:

```
rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}]"
```

* This should close all existing windows:

```
rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[]"
```

### dev\_webserver.py

A static HTTP server for the ROS share path. The location of the ROS share path is the parent directory of the `lg_common` package, which depends on which ROS `setup.bash` was sourced.

If you ran `catkin_make` and sourced `devel/setup.bash`, it will serve the `src/` path of your Catkin workspace. This is good for development.

If you ran `catkin_make install` and sourced `install/setup.bash`, it will serve the `install/share` path of your Catkin workspace. This is good for automated testing, since it verifies that `CMakeLists.txt` is complete.

If you installed `lg_common` from a `.deb` package and sourced the default `/opt/ros/indigo/setup.bash`, it will serve the system `/opt/ros/indigo/share` path. This is good for production (for now).

#### Parameters

* `port` [int] - The port on which to serve files. Default: `8008`

#### Convention for webapps

If your ROS package has a static webapp you'd like to be served, put files in the package's `webapps/` and add a snippet to the bottom of `CMakeLists.txt`:

    install(
      DIRECTORY webapps/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/webapps
    )

For example, there is an `index.html` located in `lg_common/webapps/example/` which can be accessed at `http://localhost:8008/lg_common/webapps/example/index.html`.

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

All other keyword arguments are passed on directly to the command line.

See `examples/browser.py` for an example implementation.

See `src/lg_common/managed_browser.py` for default args.

#### lg\_common.ManagedWindow

Window manager.

#### lg\_common.SceneListener

Runs a callback upon scene messages.

