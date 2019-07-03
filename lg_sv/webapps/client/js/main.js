var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var showFPS = getParameterByName('showFPS', stringToBoolean, false);
var yawOffsets = getParameterByName('yawOffsets', String, '0').split(/\s*,\s*/).map(Number);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);
var fieldOfView = getParameterByName('fov', Number, 29);
var initialPano = getParameterByName('panoid', String, '');
var rosbridgeHost = getParameterByName('rosbridgeHost', String, 'localhost');
var rosbridgePort = getParameterByName('rosbridgePort', String, '9090');
var rosbridgeSecure = getParameterByName('rosbridgeSecure', stringToBoolean, 'false');
var shouldTilt = getParameterByName('tilt', stringToBoolean, 'true');
window.devicePixelRatio = getParameterByName('pixelRatio', Number, 1.0);
// scaleFactor is fixed because it changes fov non-linearly.
// This value allows for full range of roll at 16:9.
// See js/fov_fudge.js
var scaleFactor = shouldTilt ? 2.04 : 1.0;
var scaleMatrix = [
  [scaleFactor, 0, 0, 0],
  [0, scaleFactor, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
];
var lastPov = null;

var initializeViewers = function() {
  console.log('initializing Street Viewers');

  var url = getRosbridgeUrl(rosbridgeHost, rosbridgePort, rosbridgeSecure);
  var ros = new ROSLIB.Ros({
    url: url
  });
  ros.on('connection', function() {
    console.log('ROSLIB connected');
    for (var i = 0; i < yawOffsets.length; i++) {
      initializeRes(ros, yawOffsets[i]);
    }
  });
  ros.on('error', function() {
    setTimeout(initialize, 2000);
  });
  ros.on('close', function() {
    console.error('Lost ROS connection');
    window.location.href = window.location.href;
  });
};

var initializeRes = function(ros, yawOffset) {
  var divider = document.createElement('div');
  divider.style.position = 'absolute';
  divider.style.overflow = 'hidden';
  document.body.appendChild(divider);

  var wrapper = document.createElement('div');
  wrapper.style.backgroundColor = 'black';
  wrapper.style.height = '100%';
  wrapper.style.width = '100%';
  wrapper.style.margin = '0';
  wrapper.style.padding = '0';
  divider.appendChild(wrapper);

  var canvas = document.createElement('div');
  canvas.style.backgroundColor = 'black';
  canvas.style.height = '100%';
  canvas.style.width = '100%';
  canvas.style.margin = '0';
  canvas.style.padding = '0';
  wrapper.appendChild(canvas);

  var info = document.createElement('div');
  info.style.position = 'absolute';
  info.style.bottom = '4px';
  info.style.width = '100%';
  info.style.fontFamily = 'Roboto, sans-serif';
  info.style.fontWeight = '600';
  info.style.fontSize = '9pt';
  info.style.textAlign = 'center';
  info.style.margin = '0';
  info.style.color = '#dddddd';
  divider.appendChild(info);

  var logo = document.createElement('div');
  logo.style.position = 'absolute';
  logo.style.bottom = '4px';
  logo.style.left = '4px';
  logo.style.fontFamily = 'Product Sans, sans-serif';
  logo.style.fontSize = '14pt';
  logo.style.fontWeight = '600';
  logo.style.fontStretch = 'ultra-expanded';
  logo.style.margin = '0';
  logo.style.color = '#dddddd';
  logo.innerText = 'Google';
  divider.appendChild(logo);

  if (showLinks) {
    var glEnvironment = new GLEnvironment(divider, fieldOfView, yawOffset);
    var links = new Links(glEnvironment.camera, glEnvironment.scene);
  }

  if (showFPS) {
    var stats = new Stats();
    stats.showPanel( 0 ); // 0: fps, 1: ms, 2: mb, 3+: custom
    document.body.appendChild( stats.dom );
    function animate() {
        stats.begin();
        // monitored code goes here
        stats.end();
        requestAnimationFrame( animate );
    }
    requestAnimationFrame( animate );
  }

  function handleResize() {
    var width = window.innerWidth / yawOffsets.length;
    var height = window.innerHeight;
    var index = yawOffsets.indexOf(yawOffset);
    var left = index * width;
    divider.style.width = width.toString() + 'px';
    divider.style.height = height.toString() + 'px';
    divider.style.left = left.toString() + 'px';
    if (showLinks) {
      glEnvironment.handleResize(width, height);
    }
  }
  window.addEventListener('resize', handleResize, false);
  handleResize();

  var panoTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview/panoid',
    messageType: 'std_msgs/String',
    throttle_rate: 16,
    queue_length: 1,
  });
  var metadataTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview/metadata',
    messageType: 'std_msgs/String',
    throttle_rate: 16,
    queue_length: 1,
  });

  var attributionModule = new Attribution(info);
  var handleMetadataResponse = function(response, stat) {
    if (stat != google.maps.StreetViewStatus.OK) {
      throw 'Metadata request status NOT OK: ' + stat;
    }
    attributionModule.handleMetadata(response);
    if (showLinks) {
      links.update(response);
    }
  };

  var svOptions = {
    visible: true,
    disableDefaultUI: true
  };
  var sv = new google.maps.StreetViewPanorama(canvas, svOptions);

  var svClient = new StreetviewClient(ros, sv);

  var svService = new google.maps.StreetViewService();

  function handlePanoChanged(panoId) {
    sv.setPano(panoId);
    if (shouldTilt) {
      var fovFudge = getFovFudge(fieldOfView);
      var zoomLevel = getZoomLevel(fieldOfView * scaleFactor * fovFudge);
    } else {
      var zoomLevel = getZoomLevel(fieldOfView);
    }
    sv.setZoom(zoomLevel);
    if (lastPov) {
      lastPov.w = 70;
      svClient.pubPov(lastPov);
    }
    svService.getPanorama({ pano: panoId }, handleMetadataResponse);
  }
  svClient.on('pano_changed', handlePanoChanged);
  var panoService = new ROSLIB.Service({
    ros: ros,
    name: '/streetview/panoid_state',
    serviceType: 'lg_sv/PanoIdState'
  });
  panoService.callService({}, function(resp) {
    //handlePanoChanged(resp.panoid);
  });

  /**
   * handling director scenes here
   */
  function handleDirectorMessage(scene) {
    var is_streetview = false;
    var sv_window;
    //for (_window in scene["windows"]) {
    for (var i = 0; i < scene.windows.length; i+= 1) {
      _window = scene.windows[i];
      if (_window["activity"] == "streetview") {
        is_streetview = true;
        sv_window = _window;
        break;
      }
    }
    if (is_streetview == false)
      return;

    var panoid = sv_window['activity_config']['panoid'];
    var pov = lastPov || {};
    if (sv_window['activity_config']['heading'])
      pov.z = sv_window['activity_config']['heading'];
    if (sv_window['activity_config']['tilt'])
      pov.x = sv_window['activity_config']['tilt'];


    if (panoid[0] == '-' && panoid.search("%2F") > -1)
      panoid = "F:" + panoid;

    console.log("Emitting " + panoid + " with pov " + pov);
    svClient.emit('pano_changed', panoid);
    svClient.emit('pov_changed', pov);
  }
  svClient.on('director_message', handleDirectorMessage);
  /**
   * Get the zoom level at a given horizontal fov.
   *
   * This is only correct at scaleFactor 1.0.
   *
   * @param {Number} hFov in degrees
   * @return {Number} zoom level
   */
  function getZoomLevel(hFov) {
    return -Math.log2(Math.tan(Math.PI * hFov / 360)) + 1;
  }

  /**
   * Multiply two matrices.
   *
   * <http://stackoverflow.com/a/27205341>
   *
   * @param {Array} a left matrix
   * @param {Array} b right matrix
   * @return {Array} result matrix
   */
  function multiply(a, b) {
    var aNumRows = a.length, aNumCols = a[0].length,
        bNumRows = b.length, bNumCols = b[0].length,
        m = new Array(aNumRows);  // initialize array of rows
    for (var r = 0; r < aNumRows; ++r) {
      m[r] = new Array(bNumCols); // initialize the current row
      for (var c = 0; c < bNumCols; ++c) {
        m[r][c] = 0;              // initialize the current cell
        for (var i = 0; i < aNumCols; ++i) {
          m[r][c] += a[r][i] * b[i][c];
        }
      }
    }
    return m;
  }

  /**
   * Builds the transform matrix to scale and rotate a canvas.
   *
   * @param {Array} scaleMatrix
   * @param {Number} roll in radians
   * @return {String} transform matrix CSS
   */
  function buildTransformMatrix(scaleMatrix, roll) {
    var sr = Math.sin(roll);
    var cr = Math.cos(roll);
    var rotationMatrix = [
      [cr, -sr, 0, 0],
      [sr, cr, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];

    var finalMatrix = multiply(rotationMatrix, scaleMatrix);
    var flatMatrix = Array.prototype.concat.apply([], finalMatrix);

    var cssTransform = ['matrix3d(', flatMatrix.join(), ')'].join(' ');
    return cssTransform;
  };

  var animate = function() {
    requestAnimationFrame(animate);

    var povQuaternion = lastPov;
    if (!lastPov) {
      return;
    }

    if (shouldTilt) {
      var radianOffset = toRadians(fieldOfView * yawOffset);

      var htr = [povQuaternion.z, povQuaternion.x, 0];
      var transformedHTR = transformHTR(htr, radianOffset);

      var heading = transformedHTR[0];
      var tilt = transformedHTR[1];
      var roll = transformedHTR[2];

      var rollRads = toRadians(roll);
      wrapper.style.transform = buildTransformMatrix(scaleMatrix, rollRads);
    } else {
      var heading = povQuaternion.z + yawOffset * fieldOfView;
      var tilt = 0;
      var roll = 0;
    }

    var pov = {
      heading: heading,
      pitch: tilt
    };
    sv.setPov(pov);

    if (showLinks) {
      links.handleView(povQuaternion.z, povQuaternion.x, 0, fieldOfView);
    }
  };

  /**
   * Handles an incoming pov change.
   *
   * @param {Object} povQuaternion with keys {x, y, z, w}
   **/
  var handleQuaternion = function(povQuaternion) {
    lastPov = povQuaternion;
  };

  svClient.on('pov_changed', handleQuaternion);

  if (initialPano !== '') {
    svClient.emit('pano_changed', initialPano);
  }

  if (showLinks) {
    glEnvironment.run();
  }

  animate();
};

google.maps.event.addDomListener(window, 'load', initializeViewers);

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
