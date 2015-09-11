var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var yawOffset = getParameterByName('yawOffset', Number, 0);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);
var fieldOfView = getParameterByName('fov', Number, 0);
var shouldTilt = getParameterByName('tilt', stringToBoolean, false);

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
    linksControl: showLinks
  };
  var canvas = document.getElementById('map-canvas');
  var map = new google.maps.Map(canvas, mapOptions);
  var sv = new google.maps.StreetViewPanorama(canvas, svOptions);

  map.setStreetView(sv);

  var svClient = new StreetviewClient(ros, sv);

  svClient.on('pano_changed', function(panoId) {
    console.log('Changing pano to', panoId);
    sv.setPano(panoId);
    // TODO(wjp): create zoom function
    sv.setZoom(3);
  });

  var wrapper = document.getElementById('wrapper');
  var canvasRatio = parseMatrix(
                    window.getComputedStyle(wrapper, null)
                          .getPropertyValue('transform'))[0];


  svClient.on('pov_changed', function(povQuaternion) {
    // TODO(mv): move quaternion parsing into StreetviewClient library
    var viewportFOV = fieldOfView / canvasRatio;
    var radianOffset = toRadians(viewportFOV * yawOffset);
    var htr = [povQuaternion.z, povQuaternion.x, 0];
    var transformedHTR = transformHTR(htr, radianOffset);
    var roll = -transformedHTR[2];
    var pov = {
      heading: transformedHTR[0],
      pitch: transformedHTR[1]
    };
    sv.setPov(pov);
    if (shouldTilt) {
      canvas.setAttribute('style', 'transform: rotateZ(' + roll + 'deg);');
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

function stringToBoolean(s) {
  var truePattern = /^1$|^true$/i;
  return s.search(truePattern) === 0;
}

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
