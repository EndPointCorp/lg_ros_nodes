
/*

  This function overwrites the console.log, console.debug and console.error
  functions.

  The arguments must be:
    extensionName - a string with the name of the extension
    hostName - name of the host the extension is working on, should be
               obtained with calling http://lg-head/cgi-bin/identify.py

*/

var initROSLogging = function(extensionName, hostName){

  if (hostName == null) hostName = "";
  console.log("RosLogging for " + extensionName + " at " + hostName);

  var rosLogging = new ROSLIB.Ros({
    url: 'wss://42-b:9090'
  });

  var logTopic = new ROSLIB.Topic({
    ros: rosLogging,
    name: '/logs',
    messageType: 'diagnostic_msgs/DiagnosticStatus'
  });

  // Levels are three:
  // 0 - OK
  // 1 - WARN
  // 2 - ERROR
  var sendRosLog = function(msg, level) {
    var rosMsg = new ROSLIB.Message({
      level:       level,
      name:        extensionName,
      message:     msg,
      hardware_id: hostName
    });
    logTopic.publish(rosMsg);
  };

  var old = {};

  old.log = console.log;
  console.log = function() {
    var args = Array.prototype.slice.call(arguments);
    old.log.apply(this, arguments);
    sendRosLog(args.join(' '), 0);
  };

  old.debug = console.debug;
  console.error = function() {
    var args = Array.prototype.slice.call(arguments, 1);
    old.debug.apply(this, arguments);
    sendRosLog(args.join(' '), 1);
  };

  old.error = console.error;
  console.error = function() {
    var args = Array.prototype.slice.call(arguments);
    old.error.apply(this, arguments);
    sendRosLog(args.join(' '), 2);
  };

};

