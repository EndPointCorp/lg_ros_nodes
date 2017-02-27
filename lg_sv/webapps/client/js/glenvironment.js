/**
 * A WebGL environment on top of an existing Element with transparent background.
 * @constructor
 * @param {HTMLElement} container
 * @param {Number} hFov horizontal FOV in degrees
 * @param {Number} yawOffset yaw offset in screens
 */
var GLEnvironment = function(container, hFov, yawOffset) {
  /**
   * The environment will be drawn on top of this container element.
   * @type {HTMLElement}
   */
  this.container = container;

  /**
   * Horizontal FOV in degrees.
   * @type {Number}
   */
  this.hFov = hFov;

  /**
   * Yaw offset in screens.
   * @type {Number}
   */
  this.yawOffset = yawOffset;

  /**
   * @type {THREE.Scene}
   */
  this.scene = new THREE.Scene();

  /**
   * @type {THREE.WebGLRenderer}
   */
  this.renderer = new THREE.WebGLRenderer({
    alpha: true,
    antialias: true,
    canvas: this.canvas
  });
  this.renderer.setClearColor(new THREE.Color(0x000000), 0);

  this.canvas = this.renderer.domElement;
  this.canvas.style.position = 'absolute';
  this.canvas.style.zIndex = '99999';
  this.canvas.style.top = '0px';
  this.canvas.style.left = '0px';
  this.canvas.style.backgroundColor = 'rgba(255, 255, 255, 0.0)';
  this.canvas.style.pointerEvents = 'none';
  this.container.appendChild(this.canvas);

  /**
   * @type {THREE.PerspectiveCamera}
   */
  this.camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    1,
    32768
  );
  this.scene.add(this.camera);

  this.animations_ = [];
};

/**
 * Starts running the simulation.
 **/
GLEnvironment.prototype.run = function() {
  this.lastDraw_ = Date.now();
  this.animate_();
};

/**
 * Handles a window resize.  Must be called from outside.
 * @param {Number} width of container in pixels
 * @param {Number} height of container in pixels
 */
GLEnvironment.prototype.handleResize = function(width, height) {
  this.canvas.style.width = width.toString() + 'px';
  this.canvas.style.height = height.toString() + 'px';

  this.renderer.setSize(
    width,
    height
  );

  var hFovRad = THREE.Math.degToRad(this.hFov);
  var vFovRad = 2 * Math.atan(Math.tan(hFovRad / 2) * (height / width));

  this.camera.fov = THREE.Math.radToDeg(vFovRad);
  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();

  var yawDeg = this.hFov * this.yawOffset;
  var yawRad = THREE.Math.degToRad(yawDeg);
  this.camera.rotation.set(0, 0, 0);
  this.camera.rotateY(-yawRad);
};

/**
 * Runs all registered animation methds and renders the scene.
 * @private
 */
GLEnvironment.prototype.animate_ = function() {
  var now = Date.now();

  var self = this;
  function animate_() {
    self.animate_();
  }

  requestAnimationFrame(animate_);

  // skip duplicate frames
  if (now - this.lastDraw_ < 14) {
    return;
  }

  var numAnimations = this.animations_.length;
  for (var i = 0; i < numAnimations; i++) {
    this.animations_[i]();
  }

  this.renderer.render(this.scene, this.camera);
  this.lastDraw_ = now;
};

/**
 * Registers an animation method, which will be run on each animation frame.
 * @param {function} callback
 */
GLEnvironment.prototype.addAnimation = function(callback) {
  this.animations_.push(callback);
};
