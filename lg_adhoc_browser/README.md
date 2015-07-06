## lg\_adhoc\_browser

ROS software for running and interfacing with the Chrome browser.

### Nodes

#### client

Listens to on a ROS topic, and creates, removes, and updates the browser windows.

Each ROS message needs to contain a list of all the browser windows to show. The browsers are distinguished using the `id` field.

The node manages internal list of opened browsers.

The list of browsers in the message is compared with the list of currently opened browsers, and then:

* if there is a browser, which `id` is not on the list, the browser is removed
* if there is not a browser for an `id`, then the browser is created
* if there already is a browser with the `id`, then the url and geometry are updated **[TODO: NOT IMPLEMENTED YET]**


##### Parameters

* `~viewport` [string] - name of the viewport to run at. This is a mandatory argument.

##### Subscribed Topics

* `/browser_service/<viewport>` [`lg_adhoc_browser/AdhocBrowsers`] - A list of browsers which should be opened.

### Messages

#### AdhocBrowsers

##### Fields

* `browsers` - [`AdhocBrowser[]`]

#### AdhocBrowser

##### Fields

* `id` - [`string`] - Browser id.
* `geometry` - [`lg_common/WindowGeometry`] - Geometry of the browser window.
* `url` - [`string'] - Url to be loaded into the browser.

### Example

* After a fresh start, there should be no browsers.

* This should show one window with the `endpoint.com` website:
 
  ```rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}]"
  ```

* This should show another window, with the `google.com` website:

  ```rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}, {id: '42x', geometry: {x: 600, y: 600, width: 300, height: 400}, url: 'http://google.com'}]"
  ```

* This should leave just the `endpoint.com` window shown:
  
  ```rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[{id: '42d', geometry: {x: 100, y: 200, width: 300, height: 400}, url: 'http://endpoint.com'}]"
  ```

* This should close all existing windows:

  ```rostopic pub /browser_service/superone lg_adhoc_browser/AdhocBrowsers "[]"
  ```
