var camera, scene, renderer, pano_url, mesh;
var _xTwist = 0, _yTwist = 0, _zTwist = 0;
var yawRads, pitchRads, rollRads;

var PITCH_AXIS  = new THREE.Vector3( 1,0,0 );
var YAW_AXIS    = new THREE.Vector3( 0,1,0 );
var ROLL_AXIS   = new THREE.Vector3( 0,0,1 );

pano_url = '../media/harvard-hall_6277-pano6432r.jpg';

function pano_runner() {
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

  var container, vertFov;
  var config;

  ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });

  ros.on('connection', function() {
    console.log("Connected to rosbridge");
  });

  ros.on('error', function(e) {
    console.log("rosbridge error: " + e);
  });

  ros.on('close', function() {
    console.log("Disconnected from rosbridge");
  });

  povListener = new ROSLIB.Topic({
    ros : ros,
    name : '/panoviewer/pov',
    throttle_rate: 33,
    queue_length: 1,
    messageType : 'geometry_msgs/Quaternion'
  });

  povListener.subscribe(function(msg) {
    _xTwist = msg.x;
    _yTwist = msg.y;
    _zTwist = msg.z;
  });

  panoListener = new ROSLIB.Topic({
    ros : ros,
    name : '/panoviewer/pano',
    messageType : 'std_msgs/String'
  });

  panoListener.subscribe(function(msg) {
    console.log("Received new pano: " + msg.data);
    pano_url = msg.data;
  });

  var initPublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/panoviewer/init',
    messageType : 'std_msgs/String'
  })

  // If you do this immediately, you tend not to catch the responses
  window.setTimeout( function() {
    initPublisher.publish(new ROSLIB.Message({ data: "Hello" }));
  }, 250);

  container = document.getElementById( 'container' );

  vertFov = getConfig('vertFov', 75) * 1.0;
  yawRads = toRad(getConfig('yawOffset', 0) * 1.0)
  pitchRads = toRad(getConfig('pitchOffset', 0) * 1.0)
  rollRads = toRad(getConfig('rollOffset', 0) * 1.0)

  camera = new THREE.PerspectiveCamera();
  camera.target = new THREE.Vector3(0, 0, 0);

  offsetCamera = new THREE.PerspectiveCamera(vertFov, window.innerWidth / window.innerHeight, 1, 1100 );
  camera.add(offsetCamera);

  offsetCamera.rotateOnAxis(YAW_AXIS, -yawRads);
  offsetCamera.rotateOnAxis(PITCH_AXIS, pitchRads);
  offsetCamera.rotateOnAxis(ROLL_AXIS, rollRads);

  scene = new THREE.Scene();
  mesh = getMesh();

  scene.add(mesh);
  scene.add(camera);

  renderer = new THREE.WebGLRenderer();
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );
  container.appendChild( renderer.domElement );

  window.addEventListener( 'resize', onWindowResize, false );
}

function onWindowResize() {

  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate(nowMsec) {

  requestAnimationFrame( animate );
  update(nowMsec);

}

function update(nowMsec) {
  scene.children[0] = getMesh();
  renderer.render(scene, offsetCamera);
}

function getMesh() {
  setTarget();
  if (mesh != null && mesh.material.map.sourceFile === pano_url)
    return mesh;

  mesh = new THREE.Mesh(
      new THREE.SphereGeometry(100, 32, 32),
      new THREE.MeshBasicMaterial({
        map: THREE.ImageUtils.loadTexture(pano_url)
      })
  );
  mesh.scale.x = -1;
  return mesh;
}

function setTarget() {
  if (mesh == null)
    return;
  var phi = toRad(90 - _xTwist);
  var theta = toRad(_zTwist);

  camera.target.x = 500 * Math.sin(phi) * Math.cos(theta);
  camera.target.y = 500 * Math.cos(phi);
  camera.target.z = 500 * Math.sin(phi) * Math.sin(theta);

  camera.lookAt(camera.target);
}

function toRad(deg) {
  return THREE.Math.degToRad(deg);
}
