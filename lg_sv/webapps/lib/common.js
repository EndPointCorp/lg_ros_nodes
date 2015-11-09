// Common library for webapps
// probably copywritten to endpoint

function getRosbridgeUrl(rosbridgeHost, rosbridgePort, rosbridgeSecure) {
  var url = '';

  if (rosbridgeSecure) {
    url += 'wss://';
  } else {
    url += 'ws://';
  }
  url += rosbridgeHost;
  url += ':';
  url += rosbridgePort;

  return url;
}

function stringToBoolean(s) {
  var truePattern = /^1$|^true$/i;
  return s.search(truePattern) === 0;
}

function getParameterByName(name, type, def) {
  name = name.replace(/[\[]/, '\\[').replace(/[\]]/, '\\]');
  var regex = new RegExp('[\\?&]' + name + '=([^&#]*)'),
                         results = regex.exec(location.search);
  return (results === null ? def : type(
          decodeURIComponent(results[1].replace(/\+/g, ' '))));
}
