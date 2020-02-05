class SyncedVideoApp {
  /**
   * @constructor
   * @param {Object} opts
   *
   * Opts:
   *   master: publish clock/transport from this instance?
   *   ros: ROSLIB.Ros instance
   *   sync: If true, the video should be synchronized with other player instances.  Default: false
   *   softSyncMin: min time difference for playback rate modulation, in seconds.  Default: 1.0 / 60
   *   softSyncMax: max time difference before seeking, in seconds.  Default: 1.0
   */
  constructor(opts) {
    this.master = opts.master;
    this.ros = opts.ros;
    this.sync = opts.sync;
    this.softSyncMin = opts.softSyncMin || 1.0 / 90.0;
    this.softSyncMax = opts.softSyncMax || 1.0;

    let clockAddr = opts.clockAddr || 'ws://localhost:9091';

    this.initVideo_();
    this.initClockSync_(clockAddr);
  }

  /**
   * Load a video from the given url.
   *
   * @param {String} url
   * @param {Boolean} loop
   */
  loadVideoFromUrl(url, loop) {
    this.video.loop = loop;
    this.syncedVideo.loadFromUrl(url);
  }

  /**
   * Initialize the source video element.
   *
   * @private
   */
  initVideo_() {
    this.video = document.createElement('video');
    document.body.appendChild(this.video);

    if (this.sync) {
      if (!this.master) {
        this.video.muted = true;
        this.animateVideo_ = this.animateVideoSlave_.bind(this);
      } else {
        this.animateVideo_ = this.animateVideoMaster_.bind(this);
      }
    } else {
      this.animateVideo_ = function() {};
    }

    this.syncedVideo = new SyncedVideo(this.video, this.softSyncMin, this.softSyncMax);
  }

  /**
   * Initialize network video clock sync.
   *
   * @private
   * @param {String} clockAddr address of clock WebSocket distributor
   */
  initClockSync_(clockAddr) {
    let now = performance.now();
    this.lastSeekTime_ = now / 1000 - this.softSyncMax;

    this.clockSocket = new ClockSocket(clockAddr);
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
}
