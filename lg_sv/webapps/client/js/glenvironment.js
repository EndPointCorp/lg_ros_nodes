/**
 * A WebGL environment on top of the page with transparent background.
 * @constructor
 * @param {Number} hFov horizontal FOV in degrees
 */
var GLEnvironment = function(hFov) {
  this.hFov_ = hFov;

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

  /**
   * @type {Element}
   */
  this.canvas = this.renderer.domElement;
  this.canvas.style.position = 'absolute';
  this.canvas.style.bottom = '0px';
  this.canvas.style.left = '0px';
  this.canvas.style.width = '100%';
  this.canvas.style.height = '100%';
  this.canvas.style.zIndex = '99999';
  this.canvas.style.backgroundColor = 'rgba(255, 255, 255, 0.0)';
  this.canvas.style.pointerEvents = 'none';
  document.body.appendChild(this.canvas);

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
  window.addEventListener('resize', this.handleResize_.bind(this));
  this.handleResize_();

  this.lastDraw_ = Date.now();
  this.animate_();
};

/**
 * Handles a window resize.
 * @private
 */
GLEnvironment.prototype.handleResize_ = function() {
  var width = window.innerWidth;
  var height = window.innerHeight;

  this.renderer.setSize(
    width,
    height
  );

  // Scale viewport to compensate for large canvas quirks.
  /*
  var context = this.canvas.getContext('webgl');
  this.renderer.setViewport(
    0,
    0,
    context.drawingBufferWidth / window.devicePixelRatio,
    context.drawingBufferHeight / window.devicePixelRatio
  );
  */

  var hFovRad = THREE.Math.degToRad(this.hFov_);
  var vFovRad = 2 * Math.atan(Math.tan(hFovRad / 2) * (height / width));

  this.camera.fov = THREE.Math.radToDeg(vFovRad);
  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();
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
