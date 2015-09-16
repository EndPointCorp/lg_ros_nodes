/**
 * Maximum video sync rate in Hz.
 */
var SYNC_RATE = 60;

/**
 * Followers increment the leader's time by this fixed amount to compensate for
 * frame delay.  Typically N / videoFps?
 */
var FRAME_LATENCY = 3 / 25;

/**
 * Followers send a ping at this interval.
 */
var PING_INTERVAL = 1000;

/**
 * Seek to sync if time difference is greater than this amount.
 */
var HARD_SYNC_DIFF = 1.0;

/**
 * Never set playbackRate below this.
 */
var MIN_PLAYBACKRATE = 0.5;

/**
 * Never set playbackRate above this.
 */
var MAX_PLAYBACKRATE = 1.5;

/**
 * Global ros variable
 */
var ros = null;

/**
 * Clamps a value within the min and max.
 * @param {Number} n
 * @param {Number} min
 * @param {Number} max
 * @return {Number}
 */
function clamp(n, min, max) {
  return Math.min(Math.max(n, min), max);
}

/*
 * Most of the work is done by a Popcorn plugin.
 * This turned out to be a most performant way to synchronize.
 */
(function(Popcorn) {
  Popcorn.plugin('sync', {
    _setup: function(track) {
      /**
       * Each instance gets a random id for ping.
       * @type {Number}
       */
      this.id = Math.random();

      /**
       * Most recent time value from the leader.
       */
      this.leaderPosition = 0;

      /**
       * Avoid duplicate seeks by remembering the last seek position.
       */
      this.lastSeekPosition = 0;

      /**
       * Calculate seek delay by tracking local time.
       * @type {Number}
       */
      this.lastSeekStart;

      /**
       * Running approximation of network latency for fine sync.
       */
      this.netLatency = 0;

      /**
       * The leader publishes its video transport state.
       * @type {Boolean}
       */
      this.isLeader = track.isLeader;

      ros.on('connection', function() {
        console.log('Connected');
      }.bind(this));
      ros.on('error', function(err) {
        console.log('Error connecting to bridge:', err);
      });
      ros.on('close', function() {
        console.log('Connection closed');
      });

      /**
       * Topic for video time.
       */
      this.timeTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/videosync/time',
        messageType: 'std_msgs/Float64',
        throttle_rate: Math.floor(1000 / SYNC_RATE)
      });
      /**
       * Topic for play/pause state.
       */
      this.playTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/videosync/playing',
        messageType: 'std_msgs/Bool'
      });
      /**
       * Topic for touch events.
       */
      this.touchTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/videosync/touch',
        messageType: 'std_msgs/Bool'
      });
      /**
       * Topic for ping requests.
       */
      this.pingTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/videosync/ping',
        messageType: 'std_msgs/Float64'
      });
      /**
       * Topic for ping responses.
       */
      this.pongTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/videosync/pong',
        messageType: 'std_msgs/Float64'
      });

      /**
       * Cached message for ping requests.
       */
      this.pingMsg = new ROSLIB.Message({
        data: this.id
      });

      /**
       * Time of last ping request.
       * @type {Number}
       */
      this.lastPingTime;

      /**
       * Sends a ping request.
       */
      this.sendPing = function() {
        this.lastPingTime = Date.now();
        this.pingTopic.publish(this.pingMsg);
      };

      /**
       * Sends play/pause state changes.
       */
      this.sendPlay = function() {
        var playMsg = new ROSLIB.Message({
          data: !this.paused()
        });
        this.playTopic.publish(playMsg);
      };

      /**
       * Sends touch events.
       */
      this.sendTouch = function() {
        var touchMsg = new ROSLIB.Message({
          data: true
        });
        this.touchTopic.publish(touchMsg);
      };

      /**
       * Sends time updates.
       */
      this.sendTime = function() {
        var timeMsg = new ROSLIB.Message({
          data: this.currentTime()
        });
        this.timeTopic.publish(timeMsg);
      };

      /**
       * Adjusts time for network and render latency.
       * @param {Number} t
       * @return {Number}
       */
      this.adjustForLatency = function(t) {
        return (t + this.netLatency + FRAME_LATENCY) % this.duration();
      };

      /*
       * Topic subscriptions according to leadership.
       */
      if (!this.isLeader) {
        this.timeTopic.subscribe(function(msg) {
          this.leaderPosition = this.adjustForLatency(msg.data);
        }.bind(this));

        this.playTopic.subscribe(function(msg) {
          var playing = msg.data;
          if (playing) {
            this.play();
          } else {
            this.pause();
          }
        }.bind(this));

        this.pongTopic.subscribe(function(msg) {
          var now = Date.now();
          if (msg.data == this.id) {
            this.netLatency = (now - this.lastPingTime) * 0.001 / 2;
          }
        }.bind(this));

        this.touchTopic.advertise();
        this.pingTopic.advertise();

        setInterval(this.sendPing.bind(this), PING_INTERVAL);
      } else {
        this.touchTopic.subscribe(function(msg) {
          if (this.paused()) {
            this.play();
          } else {
            this.pause();
          }
        }.bind(this));

        this.pingTopic.subscribe(function(msg) {
          this.pongTopic.publish(msg);
        }.bind(this));

        this.playTopic.advertise();
        this.timeTopic.advertise();
        this.pongTopic.advertise();

        this.sendPlay();
        this.sendTime();
      }
    },

    /*
     * Event handlers for the video follow.
     */
    start: function(ev, track) {},
    end: function(ev, track) {},

    click: function(ev) {
      // ignore touches on controls
      if (ev.offsetY < window.innerHeight - 40) {
        this.sendTouch();
      }
    },
    playing: function(ev, track) {
      if (this.isLeader) {
        this.sendPlay();
      } else {
        this.seekLatency = 0;
        this.lastSeekTime = Date.now();
      }
    },
    pause: function(ev, track) {
      if (this.isLeader) {
        this.sendPlay();
      } else {
        this.seekLatency = 0;
      }
    },
    seeked: function() {
      if (this.isLeader) {
        this.sendTime();
        this.sendPlay();
      }
    },

    /*
     * Manages synchronization within Popcorn's requestAnimationFrame cycle.
     */
    frame: function(ev, track) {
      if (this.isLeader) {
        this.sendTime();
        this.sendPlay();
      } else {
        if (this.leaderPosition != this.lastSeekPosition) {
          var diff = this.currentTime() - this.leaderPosition;

          if (Math.abs(diff) > HARD_SYNC_DIFF) {
            this.playbackRate(1.0);
            this.currentTime(this.leaderPosition);
            this.lastSeekPosition = this.leaderPosition;
            console.log('seek to:', this.leaderPosition);
          } else {
            var rate = clamp(1 - diff, MIN_PLAYBACKRATE, MAX_PLAYBACKRATE);
            this.playbackRate(rate);
          }
        }
      }
    }
  });
})(Popcorn);

/**
 * A wrapper for Popcorn videos using the 'sync' plugin.
 *
 * @param {String} vidElem The video element to populate with Popcorn.
 * @param {String} src The url of the video to play.
 * @param {Boolean} isLeader The leader has the authoritative time/transport.
 */
function SyncedVideo(vidElem, src, isLeader, rosbridge_url, opts) {
  console.log('init with src:', src, 'leader:', isLeader);
  overwriteConstants(opts);
  var srcElem = document.createElement('source');
  srcElem.src = src;
  vidElem.appendChild(srcElem);

  ros = new ROSLIB.Ros({
    url: rosbridge_url
  });

  var vid = Popcorn(vidElem, {
    frameAnimation: true
  });
  vid.preload('auto');
  vid.loop(opts['autoplay'] || false);
  vid.sync({
    isLeader: isLeader
  });

  // Avoid audible debris.
  if (!isLeader) {
    vid.mute();
  } else {
    vid.unmute();
    vid.volume(1.0);
    vid.controls(opts['show_controls'] || false);
    vid.autoplay(opts['autoplay'] || false);
  }

  return vid;
}

function overwriteConstants(opts) {
  if ('syncRate' in opts)
    SYNC_RATE = opts['syncRate'];
  if ('frameLatency' in opts)
    FRAME_LATENCY = opts['frameLatency'] * 1.0;
  if ('pingInterval' in opts)
    PING_INTERVAL = opts['pingInterval'] * 1.0;
  if ('hardSyncDiff' in opts)
    HARD_SYNC_DIFF = opts['hardSyncDiff'] * 1.0;
  if ('minPlaybackRate' in opts)
    MIN_PLAYBACKRATE = opts['minPlaybackRate']* 1.0;
  if ('maxPlaybackRate' in opts)
    MAX_PLAYBACKRATE = opts['maxPlaybackRate'] * 1.0;
}
