var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var yawOffsets = getParameterByName('yawOffsets', String, '0').split(/\s*,\s*/).map(Number);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);
var fieldOfView = getParameterByName('fov', Number, 29);
var initialPano = getParameterByName('panoid', String, '');
var rosbridgeHost = getParameterByName('rosbridgeHost', String, 'localhost');
var rosbridgePort = getParameterByName('rosbridgePort', String, '9090');
var rosbridgeSecure = getParameterByName('rosbridgeSecure', stringToBoolean, 'false');
window.devicePixelRatio = getParameterByName('pixelRatio', Number, 1.0);
var scaleFactor = 2.04;
var scaleMatrix = [
  [scaleFactor, 0, 0, 0],
  [0, scaleFactor, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
];
var lastPov = null;

var viewers = [];
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
    messageType: 'std_msgs/String'
  });
  var metadataTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview/metadata',
    messageType: 'std_msgs/String'
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

  var mapOptions = {
    disableDefaultUI: true,
    center: new google.maps.LatLng(45, 45),
    backgroundColor: 'black',
    zoom: 8
  };
  var svOptions = {
    visible: true,
    disableDefaultUI: true
  };
  var map = new google.maps.Map(canvas, mapOptions);
  var sv = new google.maps.StreetViewPanorama(canvas, svOptions);

  map.setStreetView(sv);

  var svClient = new StreetviewClient(ros, sv);

  var svService = new google.maps.StreetViewService();

  svClient.on('pano_changed', function(panoId) {
    sv.setPano(panoId);
    var fovFudge = getFovFudge(fieldOfView);
    var zoomLevel = getZoomLevel(fieldOfView * scaleFactor * fovFudge);
    sv.setZoom(zoomLevel);
    if (lastPov) {
      lastPov.w = 70;
      svClient.pubPov(lastPov);
    }
    svService.getPanorama({ pano: panoId }, handleMetadataResponse);
  });

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

    // TODO(mv): move quaternion parsing into StreetviewClient library
    var radianOffset = toRadians(fieldOfView * yawOffset);

    var htr = [povQuaternion.z, povQuaternion.x, 0];
    var transformedHTR = transformHTR(htr, radianOffset);

    var heading = transformedHTR[0];
    var tilt = transformedHTR[1];
    var roll = transformedHTR[2];

    var pov = {
      heading: heading,
      pitch: tilt
    };
    sv.setPov(pov);

    var rollRads = toRadians(roll);
    wrapper.style.transform = buildTransformMatrix(scaleMatrix, rollRads);

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
