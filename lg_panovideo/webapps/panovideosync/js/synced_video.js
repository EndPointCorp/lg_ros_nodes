class SyncedVideo extends EventEmitter2 {
  constructor(video, softSyncMin, softSyncMax) {
    super();

    this.video = video;

    this.softSyncMin = softSyncMin;
    this.softSyncMax = softSyncMax;

    this.lastSeekTime_ = Date.now();
  }

  loadFromUrl(url) {
    this.video.setAttribute('crossorigin', 'anonymous');
    this.video.src = url;

    const playPromise = this.video.play();
    playPromise.then(_ => {
    })
    .catch(err => {
      console.error(err);
    });
  }

  syncTo(t) {
    let now = Date.now();
    let tSinceSeek = now - this.lastSeekTime_;

    let masterTime = Math.max(t, 0);
    if (this.video.loop) {
      masterTime %= this.video.duration;
    } else {
      masterTime = Math.min(masterTime, this.video.duration);
    }

    let diff = this.video.currentTime - masterTime;

    if (Math.abs(diff) > this.softSyncMax && tSinceSeek > this.softSyncMax * 2) {
      this.lastSeekTime_ = now;
      this.video.currentTime = masterTime;
      this.video.playbackRate = 1.0;
    } else if (diff > this.softSyncMin) {
      this.video.playbackRate = 0.75;
    } else if (diff < -this.softSyncMin) {
      this.video.playbackRate = 1.5;
    } else {
      this.video.playbackRate = 1.0;
    }
  }
}
