/**
 * Robust convenience wrapper for ROSLIB.Ros.
 *
 * This module sets a common default websocket server url for Portal.
 *
 * All subscribe and advertise messages will be captured for replay upon
 * reconnection.
 *
 * Console log handlers are added for open, close, and error events.
 *
 * NOT supported are multiple subscriptions or advertisements to the same
 * topic by id.  Each subscription needs a single handler.
 *
 * Albatrosses are highly efficient in the air, using dynamic soaring and
 * slope soaring to cover great distances with little exertion. - Wikipedia
 *
 * @constructor
 * @param {object} options
 *       See ROSLIB.Ros for options.
 * @author Matt Vollrath <matt@endpoint.com>
 */
function AlbatRos(options) {
  options = options || {};
  options.url = options.url || 'wss://42-b:9090';
  ROSLIB.Ros.call(this, options);

  /** Debug flag.  Set to true for more output.
   *  @private */
  this.debug_ = false;

  /** Address to the rosbridge server.
   *  @private */
  this.url_ = options.url;

  /** A timer for reconnection.
   *  @private */
  this.retryTimer_ = null;

  /** Captured subscriptions for replay.
   *  @private */
  this.subscriptions_ = {};

  /** Captured advertisements for replay.
   *  @private */
  this.advertisements_ = {};

  this.on('connection', this.handleConnection_.bind(this));
  this.on('close', this.handleClose_.bind(this));
  this.on('error', this.handleError_.bind(this));
}

AlbatRos.prototype = Object.create(ROSLIB.Ros.prototype);

/**
 * Makes console output describing the rate at which messages are received
 * on a topic.
 * @param {ROSLIB.Topic} topic
 *       The topic to debug
 */
AlbatRos.prototype.debugSubscribeRate = function(topic) {
  var counter = 0;
  var topicName = topic.name;
  var topicRate = topic.throttle_rate || 0;
  var topicQueueLength = topic.queue_length || 0;
  var topicMaxHz = topicRate > 0 ? (1000 / topicRate).toFixed(2) : 'inf';
  this.on(topicName, function(msg) {
    counter += 1;
  });
  var timer = setInterval(function() {
    console.log(
      'got', counter,
      'messages on', topicName,
      'throttled at', topicRate, 'ms (' + topicMaxHz + ' Hz)',
      'queueing', topicQueueLength
    );
    counter = 0;
  }, 1000);
};

/**
 * Overrides Ros.callOnConnection(), adding advertise/subscribe capture.
 * @param {ROSLIB.Message} message
 */
AlbatRos.prototype.callOnConnection = function(message) {
  // Tuck away some messages for later.
  var captured = this.captureMessage_(message);

  // Don't pass to super callOnConnection() if we've captured this message,
  // but the socket is not yet open.  Prevents duplicate messages on first
  // connection.  The captured message will be replayed by our recurring
  // 'open' handler.
  var connected = this.socket && this.socket.readyState === WebSocket.OPEN;
  if (captured && !connected) {
    this.logDebug_('delaying message due to pre-open capture', message);
    return;
  }

  ROSLIB.Ros.prototype.callOnConnection.apply(this, arguments);
};

/**
 * Debug logging facility.  Only sends to console if this.debug_ == true.
 * @private
 */
AlbatRos.prototype.logDebug_ = function() {
  if (this.debug_) {
    console.log.apply(console, arguments);
  }
};

/**
 * Capture advertise/subscribe messages for replay.  If an unadvertise or
 * unsubscribe message is captured, remove the corresponding replay message.
 *
 * @param {ROSLIB.Message} message
 * @return {boolean} true if the message was captured and should not be passed
 *        to super.callOnConnection()
 * @private
 */
AlbatRos.prototype.captureMessage_ = function(message) {
  if (message.op == 'subscribe') {
    this.logDebug_('capturing subscription to', message.topic);
    this.subscriptions_[message.topic] = message;
    return true;

  } else if (message.op == 'unsubscribe') {
    this.logDebug_('removing captured subscription to', message.topic);
    delete this.subscriptions_[message.topic];

  } else if (message.op == 'advertise') {
    this.logDebug_('capturing advertisement for', message.topic);
    this.advertisements_[message.topic] = message;
    return true;

  } else if (message.op == 'unadvertise') {
    this.logDebug_('removing captured advertisement for', message.topic);
    delete this.advertisements_[message.topic];
  }

  return false;
};

/**
 * Replay all captured advertise/subscribe messages.
 * @private
 */
AlbatRos.prototype.replayMessages_ = function() {
  for (topic in this.subscriptions_) {
    var sub = this.subscriptions_[topic];
    this.logDebug_('replaying subscription to', topic, ':', sub);
    ROSLIB.Ros.prototype.callOnConnection.call(this, sub);
  }
  for (topic in this.advertisements_) {
    var advert = this.advertisements_[topic];
    ROSLIB.Ros.prototype.callOnConnection.call(this, advert);
    if (this.debug_) {
      console.log('replayed advertisement for', topic, ':', advert);
    }
  }
};

/**
 * Connects to rosbridge.
 * @private
 */
AlbatRos.prototype.connect_ = function() {
  this.connect(this.url_);
};

/**
 * Clears the reconnection timer.
 * @private
 */
AlbatRos.prototype.clearTimer_ = function() {
  clearInterval(this.retryTimer_);
  this.retryTimer_ = null;
};

/**
 * Starts a reconnection timer.
 * @private
 */
AlbatRos.prototype.reconnect_ = function() {
  var self = this;

  // Prevent redundant timers.
  if (this.retryTimer_ != null) {
    return;
  }

  this.retryTimer_ = setInterval(function() {
    console.log('AlbatRos attempting reconnection...');
    self.connect_();
  }, 1000);
};

/**
 * Handles the event of connection to rosbridge.
 * @private
 */
AlbatRos.prototype.handleConnection_ = function() {
  this.clearTimer_();
  this.replayMessages_();
  console.log('AlbatRos connected');
};

/**
 * Handles the event of disconnection from rosbridge.
 * @private
 */
AlbatRos.prototype.handleClose_ = function() {
  console.error('AlbatRos connection closed');
  this.reconnect_();
};

/**
 * Handles the event of a roslibjs error.
 * @private
 */
AlbatRos.prototype.handleError_ = function() {
  console.error('AlbatRos caught an error: ', arguments);
};
