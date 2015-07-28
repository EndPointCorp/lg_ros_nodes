var camera, scene, renderer, viewSyncEffect, material, pano_url, videoTexture, ws;
var _xTwist, _yTwist, _zTwist;

var isUserInteracting = false,
    onMouseDownMouseX = 0, onMouseDownMouseY = 0,
    lon = 0, onMouseDownLon = 0,
    lat = 0, onMouseDownLat = 0,
    phi = 0, theta = 0;

pano_url = 'textures/nothing-loaded.png';

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

  var container, mesh, vertFov;
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
    messageType : 'geometry_msgs/Vector3'
  });

  povListener.subscribe(function(msg) {
    _xTwist += msg.x;
    _yTwist += msg.y;
    _zTwist += msg.z;
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

  vertFov = getConfig('vertFov', 75);
  camera = new THREE.PerspectiveCamera( vertFov, window.innerWidth / window.innerHeight, 1, 1100 );
  camera.target = new THREE.Vector3( 0, 0, 0 );

  scene = new THREE.Scene();

  //                var geometry = new THREE.SphereGeometry( 500, 60, 40 );
  //                geometry.applyMatrix( new THREE.Matrix4().makeScale( -1, 1, 1 ) );
  //
  //                material = new THREE.MeshBasicMaterial( {
  //                    map: THREE.ImageUtils.loadTexture( pano_url )
  //                } );
  //
  //                mesh = new THREE.Mesh( geometry, material );

  var path = "textures/downsize/ov2_point5_50per_q90/";
  var format = '.jpg';
  var urls = [
    path + 'px' + format, path + 'nx' + format,
         path + 'py' + format, path + 'ny' + format,
         path + 'pz' + format, path + 'nz' + format
           ];

  var cubeTexture = THREE.ImageUtils.loadTextureCube( urls );
  cubeTexture.format = THREE.RGBFormat;
  //material = new THREE.MeshBasicMaterial( {
  //    map: cubeTexture,
  //    side: THREE.BackSide
  //} );

  var shader = THREE.ShaderLib[ "cube" ];
  shader.uniforms[ "tCube" ].value = cubeTexture;
  material = new THREE.ShaderMaterial( {
    fragmentShader: shader.fragmentShader,
    vertexShader: shader.vertexShader,
    uniforms: shader.uniforms,
    depthWrite: false,
    side: THREE.BackSide
  });

  mesh = new THREE.Mesh( new THREE.BoxGeometry( 100, 100, 100 ), material );

  scene.add( mesh );

  renderer = new THREE.WebGLRenderer();
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );
  container.appendChild( renderer.domElement );

  // viewSyncEffect.configure( {
  //     slave: getConfig('slave', false),
  //     pitch: getConfig('pitch', 0.0) * 1.0,
  //      roll: getConfig('roll',  0.0) * 1.0,
  //       yaw: getConfig('yaw',   0.0) * 1.0
  // });

  document.addEventListener( 'mousedown', onDocumentMouseDown, false );
  document.addEventListener( 'mousemove', onDocumentMouseMove, false );
  document.addEventListener( 'mouseup', onDocumentMouseUp, false );

  window.addEventListener( 'resize', onWindowResize, false );

}

function navigationCallback(navData) {
  var NAV_SENSITIVITY = 0.01;
  var NAV_GUTTER_VALUE = 2 * NAV_SENSITIVITY;
  for (var axis in navData.abs) {
    switch(axis) {
      case '3':
        value = navData.abs[axis] * NAV_SENSITIVITY;
        if( Math.abs( value ) > NAV_GUTTER_VALUE ) {
          lat += value;
        }
        break;
      case '5':
        value = navData.abs[axis] * NAV_SENSITIVITY;
        if( Math.abs( value ) > NAV_GUTTER_VALUE ) {
          lon += value;
        }
        break;
    }
  }
}

function loadNewTexture(url, type) {
  var texture;

  document.getElementById('info').innerText = 'Loading image...';
  console.log("Loading texture " + url + " of type " + type);
  var proxyUrl = "http://" + window.location.host + "/proxy?query=" + url;
  if (type === 'video') {
    videoTexture = new THREEx.VideoTexture(url);
    //videoTexture = new THREEx.VideoTexture(proxyUrl);
    texture = videoTexture.texture;
    texture.needsUpdate = true;
  }
  else {
    var canvas = document.createElement('canvas');
    videoTexture = undefined;
    canvas.style.position = 'absolute';
    canvas.style.top = '0';
    canvas.style.left = '0';
    texture = new THREE.Texture(canvas);

    var img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = function() {
      canvas.width = img.width;
      canvas.height = img.height;
      var context = canvas.getContext('2d');
      context.drawImage(img, 0, 0);
      texture.needsUpdate = true;
      document.getElementById('info').innerText = '';
    };
    img.src = url;
    //img.src = proxyUrl;
  }
  return texture;
}

function extraVSCallback(data) {
  if (data.type === 'pano' && data.fileurl !== pano_url) {
    material.map = loadNewTexture(data.fileurl, data.filetype);
    material.map.needsUpdate = true;
    pano_url = data.fileurl;
  }
}

function onWindowResize() {

  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize( window.innerWidth, window.innerHeight );

}

function onDocumentMouseDown( event ) {

  event.preventDefault();

  isUserInteracting = true;

  onPointerDownPointerX = event.clientX;
  onPointerDownPointerY = event.clientY;

  onPointerDownLon = lon;
  onPointerDownLat = lat;

}

function onDocumentMouseMove( event ) {

  if ( isUserInteracting === true ) {

    lon = ( onPointerDownPointerX - event.clientX ) * 0.1 + onPointerDownLon;
    lat = ( event.clientY - onPointerDownPointerY ) * 0.1 + onPointerDownLat;

  }

}

function onDocumentMouseUp( event ) {

  isUserInteracting = false;

}

/*            function onDocumentMouseWheel( event ) {

// WebKit

if ( event.wheelDeltaY ) {

camera.fov -= event.wheelDeltaY * 0.05;

// Opera / Explorer 9

} else if ( event.wheelDelta ) {

camera.fov -= event.wheelDelta * 0.05;

// Firefox

} else if ( event.detail ) {

camera.fov += event.detail * 1.0;

}

camera.updateProjectionMatrix();

} */

function animate(nowMsec) {

  requestAnimationFrame( animate );
  update(nowMsec);

}

function update(nowMsec) {

  // viewSyncEffect.extraInfo({'type': 'pano', 'fileurl' : pano_url});
  if (typeof(videoTexture) !== 'undefined') {
    videoTexture.update(.2, nowMsec - 100 / 6);
  }
  // viewSyncEffect.render( scene, camera );
  renderer.render(scene, camera);

  // if (! viewSyncEffect.isSlave()) {
  //     lat = Math.max( - 85, Math.min( 85, lat ) );
  //     phi = THREE.Math.degToRad( 90 - lat );
  //     theta = THREE.Math.degToRad( lon );

  //     camera.target.x = 500 * Math.sin( phi ) * Math.cos( theta );
  //     camera.target.y = 500 * Math.cos( phi );
  //     camera.target.z = 500 * Math.sin( phi ) * Math.sin( theta );

  //     camera.lookAt( camera.target );
  // }
}
