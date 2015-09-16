var parseQueryString = function( queryString ) {
  var params = {}, queries, temp, i, l;

  // Split into key/value pairs
  queries = queryString.split("&");

  // Convert the array of strings into an object
  for ( i = 0, l = queries.length; i < l; i++ ) {
      temp = queries[i].split('=');
      params[temp[0]] = temp[1];
  }

  return params;
};

var opts = parseQueryString(window.location.search.substr(1));

/*
 * The leader is the authoritative source of time, transport, and audio.
 */
var isLeader = opts['leader'] === 'true';

/*
 * URL of the video to play.
 */
var src = opts['src'];

/*
 * Rosbridge URL
 */
var rosbridge_url = 'ws://localhost:9090';
if ('rosbridge_host' in opts && 'rosbridge_port' in opts) {
  rosbridge_url = 'ws://' + opts['rosbridge_host'] + ':' + opts['rosbridge_port'];
}
console.assert(src, 'Need a "src" in query string');


var syncedVideo = new SyncedVideo(
  document.getElementById('vid'),
  src,
  isLeader,
  rosbridge_url,
  /* send opts to overwrite any hard coded values in syncvideo.js */
  opts
);
