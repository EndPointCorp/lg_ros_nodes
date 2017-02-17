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
    name: '/streetview/pov',
    messageType: 'geometry_msgs/Quaternion'
  });
  this.panoTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview/panoid',
    messageType: 'std_msgs/String'
  });
  this.metadataTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/streetview/metadata',
    messageType: 'std_msgs/String'
  });

  // Linkage
  this.povTopic.subscribe(this.handlePovMsg.bind(this));
  this.panoTopic.subscribe(this.handlePanoMsg.bind(this));
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

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
