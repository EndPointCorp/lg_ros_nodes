class SyncedPanoVideoApp {
  /**
   * @constructor
   * @param {Object} opts
   *
   * Opts:
   *   master: publish clock/transport from this instance?
   *   ros: ROSLIB.Ros instance
   *   hFov: horizontal field of view in degrees
   *   loop: loop video playback?  Default: false
   *   yawOffsets: array of yaw offsets in degrees
   *   softSyncMin: min time difference for playback rate modulation, in seconds.  Default: 1.0 / 60
   *   softSyncMax: max time difference before seeking, in seconds.  Default: 1.0
   */
  constructor(opts) {
    this.master = opts.master;
    this.ros = opts.ros;
    this.hFov = opts.hFov;
    this.loop = opts.loop;
    this.yawOffsets = opts.yawOffsets.map(THREE.Math.degToRad);
    this.softSyncMin = opts.softSyncMin || 1.0 / 60.0;
    this.softSyncMax = opts.softSyncMax || 1.0;

    this.initVideo_();
    this.initScene_();
    this.initCameraSync_();
    this.initClockSync_();
  }

  /**
   * Load a video from the given url.
   *
   * @param {String} url
   * @param {Object} projectionOpts
   *
   * Opts:
   *   type: projection type. can be 'cube' or 'equirectangular'
   *   expandCoef: for cube projection only. compensate transform360 expand_coef
   */
  loadVideoFromUrl(url, projectionOpts) {
    this.syncedVideo.loadFromUrl(url);
    this.scene.setProjection(projectionOpts);
  }

  /**
   * Initialize the source video element.
   *
   * @private
   */
  initVideo_() {
    this.video = document.createElement('video');
    this.video.style.display = 'none';
    document.body.appendChild(this.video);

    this.video.loop = this.loop;
    if (!this.master) {
      this.video.muted = true;
      this.animateVideo_ = this.animateVideoSlave_.bind(this);
    } else {
      this.animateVideo_ = this.animateVideoMaster_.bind(this);
    }

    this.syncedVideo = new SyncedVideo(this.video, this.softSyncMin, this.softSyncMax);
  }

  /**
   * Initialize video scene.
   *
   * @private
   */
  initScene_() {
    this.numPanels_ = this.yawOffsets.length;

    var width = window.innerWidth / this.numPanels_;
    var height = window.innerHeight;

    this.scene = new PanoVideoScene(
      this.video,
      this.hFov,
      width,
      height
    );

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
  }

  /**
   * Handle a window resize.
   */
  handleResize() {
    var panelWidth = window.innerWidth / this.numPanels_;
    var panelHeight = window.innerHeight;

    this.scene.handleResize(panelWidth, panelHeight);

    for (var i = 0; i < this.numPanels_; i++) {
      var container = this.containers_[i];
      container.style.width = panelWidth;
      container.style.height = panelHeight;
      container.style.left = i * panelWidth;
      var renderer = this.renderers_[i];
      renderer.setSize(panelWidth, panelHeight);
    }
  }

  /**
   * Initialize network camera pov sync.
   *
   * @private
   */
  initCameraSync_() {
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
  }

  /**
   * Handle a pov message from network.
   *
   * @param {Object} msg
   */
  handlePovMessage(msg) {
    this.scene.setPov(msg.z, msg.x, msg.y);
  }

  /**
   * Initialize network video clock sync.
   *
   * @private
   */
  initClockSync_() {
    let now = performance.now();
    this.lastSeekTime_ = now / 1000 - this.softSyncMax;

    this.clockSocket = new ClockSocket('ws://localhost:9091');
    this.clockSocket.open();
    this.remoteVideoClock = new RemoteVideoClock(this.clockSocket);
  }

  /**
   * Animate this instance.
   */
  animate() {
    requestAnimationFrame(() => {
      this.animate();
    });

    this.animateVideo_();
    this.animateScene_();
  }

  /**
   * Animate video playback on the master.
   *
   * @private
   */
  animateVideoMaster_() {
    this.remoteVideoClock.update(this.video.currentTime);
  }

  /**
   * Animate video playback on a slave.
   *
   * @private
   */
  animateVideoSlave_() {
    let masterTime = this.remoteVideoClock.getTime();
    this.syncedVideo.syncTo(masterTime);
  }

  /**
   * Animate the viewer.
   *
   * @private
   */
  animateScene_() {
    for (var i = 0; i < this.numPanels_; i++) {
      var yaw = this.yawOffsets[i];
      var renderer = this.renderers_[i];
      this.scene.render(renderer, yaw);
    }
  }
}
