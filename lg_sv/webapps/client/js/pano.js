var panoClient;

function panoRunner() {
  THREE.ImageUtils.crossOrigin = '';
  init();
  animate(0);
}

function getConfig(key, def) {
  key = key.replace(/[\[]/, "\\[").replace(/[\]]/, "\\]");
  var regex = new RegExp("[\\?&]" + key + "=([^&#]*)"),
      results = regex.exec(location.search);
  return results === null ? def : decodeURIComponent(results[1].replace(/\+/g, " "));
}

function init() {
  var container, vertFov, aspectRatio;
  var yawRads, pitchRads, rollRads, isLeader;
  var rosbridgeHost = getParameterByName('rosbridgeHost', String, 'localhost');
  var rosbridgePort = getParameterByName('rosbridgePort', String, '9090');
  var rosbridgeSecure = getParameterByName('rosbridgeSecure', stringToBoolean, false);
  var showAttribution = getParameterByName('showAttribution', stringToBoolean, false);
  var url = getRosbridgeUrl(rosbridgeHost, rosbridgePort, rosbridgeSecure);

  var ros = new ROSLIB.Ros({ url: url });

  container = document.getElementById( 'container' );

  vertFov = getConfig('fov', 75) * 1.0;
  aspectRatio = window.innerWidth / window.innerHeight;
  yawRads = aspectRatio * toRad(vertFov * getConfig('yawOffset', 0));
  pitchRads = toRad(getConfig('pitchOffset', 0) * 1.0);
  rollRads = toRad(getConfig('rollOffset', 0) * 1.0);
  isLeader = getConfig('leader', 'false').toLowerCase() == "true";
  initialPano = getConfig('panoid', 0);

  panoClient = new PanoClient(ros, vertFov, aspectRatio, yawRads, pitchRads,
      rollRads, isLeader, initialPano);

  container.appendChild(panoClient.getDomElement());

  window.addEventListener('resize', onWindowResize, false);

  var metadataTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/panoviewer/metadata',
    messageType: 'std_msgs/String',                                                                       
    throttle_rate: 16,
    queue_length: 1
  });
  
  var handleMetadataMsg = function(msg) {
    if (! showAttribution)
      return;

    $("#titlecard").show();
    $("#titlecard").text(JSON.parse(msg.data).location.description);
  };  
  
  var handlePanoIdMsg = function(msg) {                                                                   
    $("#titlecard").hide();
  };

  metadataTopic.subscribe(handleMetadataMsg);

}

function onWindowResize() {
  panoClient.windowResize();
}

function animate(nowMsec) {
  requestAnimationFrame(animate);
  panoClient.update(nowMsec);
}

function toRad(deg) {
  return THREE.Math.degToRad(deg);
}

window.onload = panoRunner;
