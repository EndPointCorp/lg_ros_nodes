var ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });

ros.on('connection', function() {
  console.log("Connected to rosbridge");
});

ros.on('error', function(e) {
  console.log("rosbridge error: " + e);
});

ros.on('close', function() {
  console.log("Disconnected from rosbridge");
});

povListener = new ROSLIB.Topic({
  ros : ros,
  name : '/panoviewer/pov',
  throttle_rate: 33,
  queue_length: 1,
  messageType : 'geometry_msgs/Quaternion'
});

povListener.subscribe(function(msg) {
  setTarget(msg.x, msg.y, msg.z);
});

panoListener = new ROSLIB.Topic({
  ros : ros,
  name : '/panoviewer/panoid',
  messageType : 'std_msgs/String'
});

panoListener.subscribe(function(msg) {
  console.log("Received new pano: " + msg.data);
  pano_url = msg.data;
});
