/**
 * Panoramic video viewer.
 *
 * @constructor
 * @param {Object} opts
 *
 * Opts:
 *   master: publish clock/transport from this instance?
 *   ros: ROSLIB.Ros instance
 *   hFov: horizontal field of view in degrees
 *   yawOffsets: array of yaw offsets in degrees
 *   softSyncMin: min time difference for playback rate modulation, in seconds.  Default: 1.0 / 30
 *   softSyncMax: max time difference before seeking, in seconds.  Default: 1.0
 */
function SyncedPanoVideo(opts) {
  this.master = opts.master;
  this.ros = opts.ros;
  this.hFov = opts.hFov;
  this.yawOffsets = opts.yawOffsets.map(THREE.Math.degToRad);
  this.softSyncMin = opts.softSyncMin || 1.0 / 30.0;
  this.softSyncMax = opts.softSyncMax || 1.0;

  this.initVideo_();
  this.initViewer_();
  this.initCameraSync_();
  this.initClockSync_();
}

/**
 * Axis for yaw rotation.
 */
SyncedPanoVideo.YAW_AXIS = new THREE.Vector3(0, 1, 0);

/**
 * Load a video from the given url.
 *
 * @param {String} url
 */
SyncedPanoVideo.prototype.loadVideoFromUrl = function(url) {
  this.video.setAttribute('crossorigin', 'anonymous');
  this.video.src = url;
  this.video.load();
};

/**
 * Initialize the source video element.
 *
 * @private
 */
SyncedPanoVideo.prototype.initVideo_ = function() {
  this.video = document.createElement('video');
  //this.video.loop = true;
  this.video.style.display = 'none';

  var self = this;
  this.video.addEventListener('canplaythrough', function() {
    self.video.play();
  });
};

/**
 * Convert horizontal fov to vertical fov for given viewport size.
 *
 * @private
 * @param {Number} hFov in degrees
 * @param {Number} width of viewport
 * @param {Number} height of viewport
 * @return {Number} vertical fov in degrees
 */
SyncedPanoVideo.flipFov_ = function(hFov, width, height) {
  var hFovRad = THREE.Math.degToRad(hFov);
  var vFovRad = 2 * Math.atan(Math.tan(hFovRad / 2) * (height / width));
  return THREE.Math.radToDeg(vFovRad);
};

/**
 * Initialize sphere viewer.
 *
 * @private
 */
SyncedPanoVideo.prototype.initViewer_ = function() {
  if (!this.master) {
    this.video.muted = true;
  }

  this.numPanels_ = this.yawOffsets.length;

  this.scene_ = new THREE.Scene();

  var width = window.innerWidth / this.yawOffsets.length;
  var height = window.innerHeight;
  var aspect = width / height;
  var near = 0.1;
  var far = 2000;
  var fov = SyncedPanoVideo.flipFov_(this.hFov, width, height);
  this.camera_ = new THREE.PerspectiveCamera(fov, aspect, near, far);
  this.camera_.target = new THREE.Vector3(0, 0, 0);
  this.scene_.add(this.camera_);

  var sphereGeometry = new THREE.SphereGeometry(1000, 64, 64);
  sphereGeometry.scale(-1, 1, 1);
  sphereGeometry.rotateY(Math.PI);
  var texture = new THREE.VideoTexture(
    this.video,
    THREE.UVMapping,
    THREE.ClampToEdgeWrapping,
    THREE.ClampToEdgeWrapping,
    THREE.LinearFilter,
    THREE.LinearFilter
  );
  var sphereMaterial = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.DoubleSide
  });
  var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
  this.scene_.add(sphere);

  this.domElement = document.createElement('div');
  this.containers_ = [];
  this.renderers_ = [];
  for (var i = 0; i < this.numPanels_; i++) {
    var container = document.createElement('div');
    container.style.position = 'absolute';
    container.style.width = width;
    container.style.height = height;
    container.style.left = i * width;
    this.containers_.push(container);
    this.domElement.appendChild(container);
    var renderer = new THREE.WebGLRenderer();
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);
    this.renderers_.push(renderer);
  }

  window.addEventListener('resize', this.handleResize.bind(this), false);
  this.handleResize();
};

/**
 * Handle a window resize.
 */
SyncedPanoVideo.prototype.handleResize = function() {
  var width = window.innerWidth / this.yawOffsets.length;
  var height = window.innerHeight;
  var aspect = width / height;
  var fov = SyncedPanoVideo.flipFov_(this.hFov, width, height);
  this.camera_.aspect = aspect;
  this.camera_.fov = fov;
  this.camera_.updateProjectionMatrix();
  for (var i = 0; i < this.numPanels_; i++) {
    var container = this.containers_[i];
    container.style.width = width;
    container.style.height = height;
    container.style.left = i * width;
    var renderer = this.renderers_[i];
    renderer.setSize(width, height);
  }
};

/**
 * Initialize network camera pov sync.
 *
 * @private
 */
SyncedPanoVideo.prototype.initCameraSync_ = function() {
  this.pov_ = [0, 0, 0];
  this.poseTopic_ = new ROSLIB.Topic({
    ros: this.ros,
    name: '/panovideo/pov',
    messageType: 'geometry_msgs/Quaternion'
  });
  this.poseTopic_.subscribe(this.handlePovMessage.bind(this));
  if (this.master) {
    // XXX: This is backwards.
    // We need to initialize the camera to zero when a new video is loaded.
    // This should really come from the backend, which has not been written yet.
    this.poseTopic_.advertise();
    this.poseTopic_.publish(new ROSLIB.Message({
      x: 0,
      y: 0,
      z: 0,
      w: 0
    }));
    // XXX: This is backwards.
    // We need to signal the SpaceNav "server" that the application is visible.
    // This should really come from the backend, which has not been written yet.
    // Also, this is racing against a HIDDEN state on the topic from the backend.
    this.stateTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/panovideo/state',
      messageType: 'lg_common/ApplicationState'
    });
    this.stateTopic.advertise();
    this.stateTopic.publish(new ROSLIB.Message({
      state: 'VISIBLE'
    }));
  }
};

/**
 * Handle a pov message from network.
 *
 * @param {Object} msg
 */
SyncedPanoVideo.prototype.handlePovMessage = function(msg) {
  this.pov_[0] = msg.z;
  this.pov_[1] = msg.x;
  this.pov_[2] = msg.y;
};

/**
 * Initialize network video clock sync.
 *
 * @private
 */
SyncedPanoVideo.prototype.initClockSync_ = function() {
  var self = this;
  this.masterTime_ = 0;
  this.lastSeekTime_ = window.performance.now() / 1000 - this.softSyncMax;
  this.lastClockUpdate_ = window.performance.now();
  this.clockSocket = new WebSocket('ws://localhost:9091');
  this.clockSocket.onmessage = function(ev) {
    self.lastClockUpdate_ = ev.timeStamp / 1000;
    self.masterTime_ = Number(ev.data);
  };
};

/**
 * Animate this instance.
 */
SyncedPanoVideo.prototype.animate = function() {
  var self = this;
  requestAnimationFrame(function() {
    self.animate();
  });

  this.animateVideo_();
  this.animateScene_();
};

/**
 * Animate the video playback.
 *
 * @private
 */
SyncedPanoVideo.prototype.animateVideo_ = function() {
  if (this.master) {
    this.masterTime_ = this.video.currentTime;
    this.clockSocket.send(this.masterTime_.toString());
    return;
  }

  var now = window.performance.now() / 1000;
  var interpMasterTime = this.masterTime_ + (now - this.lastClockUpdate_);
  var diff = this.video.currentTime - interpMasterTime;
  var tSinceSeek = now - this.lastSeekTime_;
  if (Math.abs(diff) > this.softSyncMax && tSinceSeek > this.softSyncMax * 2) {
    this.lastSeekTime_ = now;
    this.video.currentTime = interpMasterTime;
    this.video.playbackRate = 1.0;
    console.log('seek', diff);
  } else if (diff > this.softSyncMin) {
    this.video.playbackRate = 0.75;
    console.log('speed down', diff);
  } else if (diff < -this.softSyncMin) {
    this.video.playbackRate = 1.5;
    console.log('speed up', diff);
  } else {
    this.video.playbackRate = 1.0;
    console.log('level', diff);
  }
};

/**
 * Animate the viewer.
 *
 * @private
 */
SyncedPanoVideo.prototype.animateScene_ = function() {
  var phi = THREE.Math.degToRad(90 - this.pov_[1]);
  var theta = THREE.Math.degToRad(this.pov_[0]);
  var sp = Math.sin(phi);
  var cp = Math.cos(phi);
  var st = Math.sin(theta);
  var ct = Math.cos(theta);
  this.camera_.target.x = 500 * sp * ct;
  this.camera_.target.y = 500 * cp;
  this.camera_.target.z = 500 * sp * st;
  for (var i = 0; i < this.numPanels_; i++) {
    var yaw = this.yawOffsets[i];
    var renderer = this.renderers_[i];
    this.camera_.lookAt(this.camera_.target);
    this.camera_.rotateOnAxis(SyncedPanoVideo.YAW_AXIS, -yaw);
    renderer.render(this.scene_, this.camera_);
  }
};
