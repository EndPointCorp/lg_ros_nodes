var camera, scene, renderer, pano_url, mesh;
var yawRads, pitchRads, rollRads;
var videoTexture, last_pano_url;

var PITCH_AXIS  = new THREE.Vector3( 1,0,0 );
var YAW_AXIS    = new THREE.Vector3( 0,1,0 );
var ROLL_AXIS   = new THREE.Vector3( 0,0,1 );

pano_url = '../media/Avicii.mp4';

function panoRunner() {
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

  container = document.getElementById( 'container' );

  vertFov = getConfig('vertFov', 75) * 1.0;
  yawRads = toRad(vertFov * getConfig('yawOffset', 0));
  pitchRads = toRad(getConfig('pitchOffset', 0) * 1.0);
  rollRads = toRad(getConfig('rollOffset', 0) * 1.0);

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

var lastTimeMsec = null;
function update(nowMsec) {
  lastTimeMsec  = lastTimeMsec || nowMsec-1000/60
  var deltaMsec = Math.min(200, nowMsec - lastTimeMsec)
  lastTimeMsec  = nowMsec

  scene.children[0] = getMesh();
  if (videoTexture != null) {
    videoTexture.update(deltaMsec / 1000, nowMsec / 1000);
  }
  renderer.render(scene, offsetCamera);
}

function getMesh() {
  if (mesh != null && last_pano_url === pano_url)
    return mesh;
  last_pano_url = pano_url;

  if (videoTexture != null)
    videoTexture.destroy();

  var material;
  if (pano_url.search(/.mp4/) != -1) {
    videoTexture = new THREEx.VideoTexture(pano_url);
    material = videoTexture.texture;
    updateVideo = videoTexture.update;
    videoTexture.video.loop = false;
  } else {
    material = THREE.ImageUtils.loadTexture(pano_url);
    videoTexture = null;
  }
  mesh = new THREE.Mesh(
      new THREE.SphereGeometry(100, 32, 32),
      new THREE.MeshBasicMaterial({
        map: material
      })
  );
  mesh.scale.x = -1;
  return mesh;
}

function setTarget(xTwist, yTwist, zTwist) {
  if (mesh == null)
    return;
  var phi = toRad(90 - xTwist);
  var theta = toRad(zTwist);

  camera.target.x = 500 * Math.sin(phi) * Math.cos(theta);
  camera.target.y = 500 * Math.cos(phi);
  camera.target.z = 500 * Math.sin(phi) * Math.sin(theta);

  camera.lookAt(camera.target);
}

function toRad(deg) {
  return THREE.Math.degToRad(deg);
}

window.onload = panoRunner;
