function getParameterByName(name, type, def) {
  name = name.replace(/[\[]/, '\\[').replace(/[\]]/, '\\]');
  var regex = new RegExp('[\\?&]' + name + '=([^&#]*)'),
                         results = regex.exec(location.search);
  return (results === null ? def : type(
          decodeURIComponent(results[1].replace(/\+/g, ' '))));
}

function isTrue(val) {
  return val === '1' ||
    val === 1 ||
    val === true ||
    ('' + val).toLowerCase() === 'true';
}

const videoUrl = getParameterByName('videoUrl', String, null);
const projection = getParameterByName('projection', String, 'equirectangular');
const expandCoef = getParameterByName('expandCoef', Number, 1.0);
const master = getParameterByName('master', Boolean, false);
const hFov = getParameterByName('hFov', Number, 35);
const loop = getParameterByName('loop', isTrue, false);
const yawOffsets = getParameterByName('yawOffsets', String, '0')
  .split(/\s*,\s*/)
  .map(Number);
const rosbridgeHost = getParameterByName('rosbridge_host', String, 'localhost');
const rosbridgePort = getParameterByName('rosbridge_port', Number, 9090);
const rosbridgeSecure = getParameterByName('rosbridge_secure', isTrue, false);
const clockAddr = getParameterByName('clockAddr', String, 'ws://42-a:9091');

const protocol = (rosbridgeSecure ? 'wss' : 'ws') + '://';
const rosbridgeUrl = protocol + rosbridgeHost + ':' + rosbridgePort;

var ros = new ROSLIB.Ros();
ros.connect(rosbridgeUrl);

var sync = new SyncedPanoVideoApp({
    master: master,
    ros: ros,
    clockAddr: clockAddr,
    hFov: hFov,
    loop: loop,
    yawOffsets: yawOffsets
});
document.body.appendChild(sync.domElement);

if (videoUrl) {
  let projectionOpts = {
    type: projection,
    expandCoef: expandCoef
  };
  sync.loadVideoFromUrl(videoUrl, projectionOpts, loop);
}

sync.animate();

function handleScene(data) {
  console.log(data);
  let projectionOpts = {
    type: 'equirectangular',
    expandCoef: 1.025
  };
  let loop = false;
  let w = data['windows'][0];
  let config = w['activity_config'] || {};
  if (config.hasOwnProperty('projection')) {
    projectionOpts['type'] = config['projection'];
  }
  if (config.hasOwnProperty('expandCoef')) {
    projectionOpts['expandCoef'] = config['expandCoef'];
  }
  if (config.hasOwnProperty('loop')) {
    loop = config['loop'];
  }
  let videoUrl = w.assets[0];
  sync.loadVideoFromUrl(videoUrl, projectionOpts, loop);
}

let sceneService = new ROSLIB.Service({
  ros: ros,
  name: '/uscs/message',
  serviceType: 'lg_common/USCSService'
});
let sceneRequest = new ROSLIB.ServiceRequest({});
sceneService.callService(sceneRequest, (result) => {
  if (result.type !== 'json') {
    console.warn('Non-JSON response from USCS service');
    return;
  }
  let sceneData = JSON.parse(result.message);
  handleScene(sceneData);
});
let sceneTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/director/scene',
  messageType: 'interactivespaces_msgs/GenericMessage'
});
sceneTopic.subscribe((msg) => {
  if (msg.type !== 'json') {
    console.error('Non-JSON message on USCS topic');
    return;
  }
  let sceneData = JSON.parse(msg.message);
  handleScene(sceneData);
});
