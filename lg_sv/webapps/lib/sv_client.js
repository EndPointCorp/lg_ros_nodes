/**
 * Street View client library.
 *
 * Depends upon EventEmitter2 and ROSLIB.
 *
 * @constructor
 * @param {ROSLIB.Ros} ros
 * @param {google.maps.StreetViewPanorama} streetView
 */

function StreetviewClient(ros, streetView) {
  this.streetView = streetView;

  this.povTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview_old/pov',
    messageType: 'geometry_msgs/Quaternion',
    throttle_rate: 16,
    queue_length: 1,
  });
  this.panoTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview_old/panoid',
    messageType: 'std_msgs/String',
    throttle_rate: 16,
    queue_length: 1,
  });
  this.metadataTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview_old/metadata',
    messageType: 'std_msgs/String',
    throttle_rate: 16,
    queue_length: 1,
  });

  // Linkage
  this.povTopic.subscribe(this.handlePovMsg.bind(this));
  //this.panoTopic.subscribe(this.handlePanoMsg.bind(this));
  this.init_last_view(ros);
  this.metadataTopic.advertise();
  this.streetView.addListener('links_changed', this.sendMetadata.bind(this));
}

StreetviewClient.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Handles a pov message from the ROS graph.
 * @param {object} msg
 *       geometry_msgs/Quaternion representation of pov
 */
StreetviewClient.prototype.handlePovMsg = function(msg) {
  this.emit('pov_changed', msg);
};

/**
 * Handles a pano message from the ROS graph.
 * @param {object} msg
 *       std_msgs/String representation of panoId
 */
StreetviewClient.prototype.handlePanoMsg = function(msg) {
  var panoId = msg.data;
  this.emit('pano_changed', panoId);
};

/**
 * Publishes metadata for the current pano to the ROS graph.
 */
StreetviewClient.prototype.sendMetadata = function() {
  var links = this.streetView.getLinks();
  var panoLocation = this.streetView.getLocation();
  var photographerPov = this.streetView.getPhotographerPov();
  var metadata = {
    links: [],
    location: {
      latLng: {
        lat: panoLocation.latLng.lat(),
        lng: panoLocation.latLng.lng()
      },
      pano: panoLocation.pano,
      description: panoLocation.description,
      shortDescription: panoLocation.shortDescription
    },
    photographerPov: {
      heading: photographerPov.heading,
      pitch: photographerPov.pitch
    }
  };
  for (var i = 0; i < links.length; i++) {
    var link = links[i];
    metadata.links.push({
      description: link.description,
      heading: link.heading,
      pano: link.pano
    });
  }
  var msg = new ROSLIB.Message({
    data: JSON.stringify(metadata)
  });
  this.metadataTopic.publish(msg);
};

/**
 * Publish a new pov, useful when trying to zoom out during pano change
 */
StreetviewClient.prototype.pubPov = function(pov) {
  this.povTopic.publish(pov);
};

/**
 * Initialize this instance of streetview with the last known
 * streetview image, found by director message
 */
StreetviewClient.prototype.init_last_view = function(ros) {
  var director = new ROSLIB.Service({
    ros: ros,
    name: '/uscs/message',
    serviceType: 'lg_msg_defs/USCSMessage'
  });

  var directorTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/director/scene',
    serviceType: 'interactivespaces_msgs/GenericMessage',
    throttle_rate: 16,
    queue_length: 1,
  });

  function handleDirector(msg) {
    console.log("message is " + msg);
    scene = JSON.parse(msg.message);
    console.log("scene is " + scene);
    console.log("message.msg is " + msg.message);
    this.emit('director_message', scene);
  };
  director.callService({}, handleDirector.bind(this));
  directorTopic.subscribe(handleDirector.bind(this));
};

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
