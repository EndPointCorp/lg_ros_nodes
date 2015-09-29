var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var yawOffset = getParameterByName('yawOffset', Number, 0);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);
var fieldOfView = getParameterByName('fov', Number, 0);
var shouldTilt = getParameterByName('tilt', stringToBoolean, false);
var scaleX = getParameterByName('scaleX', Number, 1.66);
var scaleY = getParameterByName('scaleY', Number, 1.66);
var scaleZ = getParameterByName('scaleZ', Number, 1);

function initialize() {
  console.log('initializing Street View');

  var ros = new ROSLIB.Ros({
    // TODO (wz) parametrize this
    url: 'ws://localhost:9090'
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
    linksControl: showLinks,
    mode: 'html5'
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
    sv.setZoom(3);
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
    viewportFOV = get_fov2(get_zoom(povQuaternion.y)) / canvasRatio;
    console.log(viewportFOV);
    var radianOffset = toRadians(viewportFOV * yawOffset);
    var htr = [povQuaternion.z, povQuaternion.x, 0];
    if (! shouldTilt) {
      htr[1] = 0;
    }
    var transformedHTR = transformHTR(htr, radianOffset);
    var roll = -transformedHTR[2];
    var pov = {
      heading: transformedHTR[0],
      pitch: transformedHTR[1],
      zoom: get_zoom(povQuaternion.y)
    };
    sv.setPov(pov);
    if (shouldTilt) {
      canvas.css('transform', 'rotateZ(' + roll + 'deg);');
    }
  });
}

function get_fov(z) {
  var fov = 180 / Math.pow(2, z);
  return fov;
}

function get_fov2(z) {
  console.log('z incoming: ' + z);
  var fovs = [127, 90, 53, 28, 14];
  var floor = Math.floor(z);

  if (floor < 0)
    return fovs[0];
  if (floor >= fovs.length - 1)
    return fovs[fovs.length - 1];

  var fov = fovs[floor];
  var diff = fovs[floor] - fovs[floor + 1];
  var extra = z - Math.floor(z);
  return fov - diff * extra;
}

function get_zoom(z) {
  var default_zoom = 3;
  var default_factor = 250;
  var min_zoom = 0;
  var max_zoom = 4;
  var ret = default_zoom - z / default_factor;

  if (ret < min_zoom)
    ret = min_zoom;

  if (ret > max_zoom)
    ret = max_zoom;

  return ret;
}

google.maps.event.addDomListener(window, 'load', initialize);

function getParameterByName(name, type, def) {
  name = name.replace(/[\[]/, '\\[').replace(/[\]]/, '\\]');
  var regex = new RegExp('[\\?&]' + name + '=([^&#]*)'),
                         results = regex.exec(location.search);
  return (results === null ? def : type(
          decodeURIComponent(results[1].replace(/\+/g, ' '))));
}

function stringToBoolean(s) {
  var truePattern = /^1$|^true$/i;
  return s.search(truePattern) === 0;
}

function getScaleString() {
  var ret = 'scale3d({x}, {y}, {z})';

  return ret.replace('{x}', scaleX).replace('{y}', scaleY).replace('{z}', scaleZ);
}

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
