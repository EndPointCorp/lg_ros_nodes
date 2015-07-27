function getParameterByName(name, type, def) {
  name = name.replace(/[\[]/, "\\[").replace(/[\]]/, "\\]");
  var regex = new RegExp("[\\?&]" + name + "=([^&#]*)"),
    results = regex.exec(location.search);
  return results === null ? def : type(decodeURIComponent(results[1].replace(/\+/g, " ")));
}

function stringToBoolean(s) {
  var truePattern = /^1$|^true&/i;
  return s.search(truePattern) === 0;
}

var showLinks = getParameterByName('showLinks', stringToBoolean, false);
var yawOffset = getParameterByName('yawOffset', Number, 0);
var pitchOffset = getParameterByName('pitchOffset', Number, 0);

function wrap(val, low, high) {
  if (val > high) {
    val = val - (high - low);
  }
  if (val < low) {
    val = val + (high - low);
  }
  return val;
}

function initialize() {
  console.log('initializing Street View');

  var canvas = document.getElementById('map-canvas');

  var mapOptions = {
    disableDefaultUI: true,
    center: new google.maps.LatLng(45,45),
    backgroundColor: "black",
    zoom: 8
  };
  var map = new google.maps.Map(canvas, mapOptions);

  var svOptions = {
    visible: true,
    disableDefaultUI: true,
    linksControl: showLinks
  };
  var sv = new google.maps.StreetViewPanorama(canvas, svOptions);

  map.setStreetView(sv);

  var ros = new ROSLIB.Ros({
    // TODO (wz) parametrize this
    url: 'ws://localhost:9090'
  });
  ros.on('connection', function() {
    console.log('ROSLIB connected');
  });

  var svClient = new StreetviewClient(ros, sv);

  svClient.on('pano_changed', function(panoId) {
    console.log('Changing pano to', panoId);
    sv.setPano(panoId);
  });

  svClient.on('pov_changed', function(povQuaternion) {
    // TODO(mv): move quaternion parsing into StreetviewClient library
    // TODO(wjp): Set FOV somewhere outside of the client
    var placeHolderFOV = 33;
    var newHeading = wrap(yawOffset*placeHolderFOV+povQuaternion.z, 0, 360);
    var pov = {
      heading: newHeading,
      pitch: povQuaternion.x
    };
    var roll = povQuaternion.y;
    var zoom = povQuaternion.w;
    console.log('Changing pov to', pov, roll, zoom);
    sv.setPov(pov);
    // TODO(wjp): create zoom function
    sv.setZoom(3);
  });
}

google.maps.event.addDomListener(window, 'load', initialize);

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
