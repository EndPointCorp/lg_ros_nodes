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

