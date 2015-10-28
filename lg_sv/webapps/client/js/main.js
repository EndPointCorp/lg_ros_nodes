var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var yawOffset = getParameterByName('yawOffset', Number, 0);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);
var fieldOfView = getParameterByName('fov', Number, 0);
var shouldTilt = getParameterByName('tilt', stringToBoolean, false);
var shouldZoom = getParameterByName('zoom', stringToBoolean, false);
var initialZoom = getParameterByName('initialZoom', Number, 3);
var scaleX = getParameterByName('scaleX', Number, 1.66);
var scaleY = getParameterByName('scaleY', Number, 1.66);
var scaleZ = getParameterByName('scaleZ', Number, 1);
var rosbridgeHost = getParameterByName('rosbridgeHost', String, 'localhost');
var rosbridgePort = getParameterByName('rosbridgePort', String, '9090');
var rosbridgeSecure = getParameterByName('rosbridgeSecure', stringToBoolean, 'false');

function initialize() {
  console.log('initializing Street View');

  var url = getRosbridgeUrl(rosbridgeHost, rosbridgePort, rosbridgeSecure);
  var ros = new ROSLIB.Ros({
    url: url
  });
  ros.on('connection', function() {
    console.log('ROSLIB connected');
  });

  var mapOptions = {
    disableDefaultUI: true,
    center: new google.maps.LatLng(45, 45),
    backgroundColor: 'black',
    zoom: 8
  };
  var svOptions = {
    visible: true,
    disableDefaultUI: true,
    linksControl: showLinks
  };
  var canvas = $('#map-canvas');
  var map = new google.maps.Map(canvas[0], mapOptions);
  var sv = new google.maps.StreetViewPanorama(canvas[0], svOptions);

  map.setStreetView(sv);

  var svClient = new StreetviewClient(ros, sv);

  svClient.on('pano_changed', function(panoId) {
    console.log('Changing pano to', panoId);
    sv.setPano(panoId);
    // TODO(wjp): create zoom function
    sv.setZoom(initialZoom);
  });

  var canvasRatio = 1;
  var wrapper = $('#wrapper');
  if (shouldTilt) {
    wrapper.css('transform', getScaleString());
    canvasRatio = parseMatrix(wrapper.css('transform'))[0];
  }

  svClient.on('pov_changed', function(povQuaternion) {
    // TODO(mv): move quaternion parsing into StreetviewClient library
    var viewportFOV = fieldOfView / canvasRatio;
    var radianOffset = toRadians(viewportFOV * yawOffset);
    var htr = [povQuaternion.z, povQuaternion.x, 0];
    if (! shouldTilt) {
      htr[1] = 0;
    }
    var transformedHTR = transformHTR(htr, radianOffset);
    var roll = -transformedHTR[2];
    var pov = {
      heading: transformedHTR[0],
      pitch: transformedHTR[1]
    };
    sv.setPov(pov);
    if (shouldTilt) {
      canvas.css('transform', 'rotateZ(' + roll + 'deg);');
    }
  });
}

google.maps.event.addDomListener(window, 'load', initialize);

function getParameterByName(name, type, def) {
  name = name.replace(/[\[]/, '\\[').replace(/[\]]/, '\\]');
  var regex = new RegExp('[\\?&]' + name + '=([^&#]*)'),
                         results = regex.exec(location.search);
  return (results === null ? def : type(
          decodeURIComponent(results[1].replace(/\+/g, ' '))));
}

function getScaleString() {
  var ret = 'scale3d({x}, {y}, {z})';

  return ret.replace('{x}', scaleX).replace('{y}', scaleY).replace('{z}', scaleZ);
}

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
