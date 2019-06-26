class RemoteVideoClock {
  constructor(clockSocket) {
    let now = performance.now();

    this.clockSocket_ = clockSocket;
    this.masterTime_ = 0;
    this.lastClockUpdate_ = now;

    this.clockSocket_.on('incoming', (data, t) => {
      this.lastClockUpdate_ = t;
      this.masterTime_ = data;
    });
  }

  getTime() {
    let now = performance.now();

    let elapsedSeconds = (now - this.lastClockUpdate_) / 1000;
    return this.masterTime_ + elapsedSeconds;
  }

  update(t) {
    let now = performance.now();

    this.clockSocket_.send(t);

    this.lastClockUpdate_ = now;
    this.masterTime_ = t;
  }
}
