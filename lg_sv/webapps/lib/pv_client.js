function PanoClient(ros, vertFov, aspectRatio, yawRads, pitchRads, rollRads,
    isLeader, initialPano) {
  this.lastTimeMsec = null;

  this.MIN_PLAYBACKRATE = 0.5;

  this.MAX_PLAYBACKRATE = 1.5;

  this.HARD_SYNC_DIFF = 1;

  this.PITCH_AXIS = new THREE.Vector3(1,0,0);

  this.YAW_AXIS = new THREE.Vector3(0,1,0);

  this.ROLL_AXIS = new THREE.Vector3(0,0,1);

  this.pano_url = '../media/foo.jpg';
  if (initialPano !== 0) {
    this.pano_url = initialPano;
  }

  this.leaderPosition = null;

  this.isLeader = isLeader;

  ros.on('connection', function() {
    console.log("Connected to rosbridge");
  });

  ros.on('error', function(e) {
    console.log("rosbridge error: " + e);
  });

  ros.on('close', function() {
    console.log("Disconnected from rosbridge");
    window.location.href = window.location.href;
  });

  this.povListener = new ROSLIB.Topic({
    ros : ros,
    name : '/panoviewer/pov',
    throttle_rate: 16,
    queue_length: 1,
    messageType : 'geometry_msgs/Quaternion'
  });

  this.povListener.subscribe(function(msg) {
    this.setTarget(msg.x, msg.y, msg.z);
  }.bind(this));

  this.panoListener = new ROSLIB.Topic({
    ros : ros,
    name : '/panoviewer/panoid',
    messageType : 'std_msgs/String',
    throttle_rate : 16,
    queue_length : 1,
  });

  this.panoHandler = function(msg) {
    console.log("Received new pano: " + msg.data);
    this.pano_url = msg.data;
    /* set last_pano to null so duplicate videos start over */
    this.last_pano_url = null;
  };
  this.panoListener.subscribe(this.panoHandler.bind(this));

  this.directorListener = new ROSLIB.Topic({
    ros: ros,
    name: '/director/scene',
    throttle_rate: 16,
    queue_length: 1,
    messageType: 'interactivespaces_msgs/GenericMessage'
  });

  this.handleDirectorMessage = function(msg) {
    var scene = JSON.parse(msg.message);
    var is_panoview = false;
    var pv_window;
    for (var i = 0; i < scene.windows.length; i+=1) {
      var _window = scene.windows[i];
      if (_window["activity"] == "panoviewer") {
        is_panoview = true;
        pv_window = _window;
        break;
      }
    }
    if (is_panoview == false)
      return false;

    /*
     * TODO: when we start using the activity_config for panoids for the
     * panoviewer we should change pv_window['assets'][0] to
     * pv_window['activity_config']['panoid'] and also grab the POV section if
     * it exists as we do in the sv_client.js
     */
    var panoid = pv_window['assets'][0];
    this.panoHandler({ data: panoid });
  };
  this.directorListener.subscribe(this.handleDirectorMessage.bind(this));
  this.stateTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/panoviewer/state',
    messageType: 'lg_msg_defs/ApplicationState',
    throttle_rate: 16,
    queue_length: 1
  });

  this.timeTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/panoviewer/video_time',
    messageType: 'std_msgs/Float64',
    throttle_rate: 16,
    queue_length: 1
  });

  this.timeTopic.subscribe(function(msg) {
    this.leaderPosition = msg.data;
  }.bind(this));

  this.stateTopic.subscribe(this.handleState.bind(this));


  this.camera = new THREE.PerspectiveCamera();
  this.camera.target = new THREE.Vector3(0, 0, 0);

  this.offsetCamera = new THREE.PerspectiveCamera(vertFov, aspectRatio, 1, 1100 );
  this.camera.add(this.offsetCamera);

  this.offsetCamera.rotateOnAxis(this.YAW_AXIS, -yawRads);
  this.offsetCamera.rotateOnAxis(this.PITCH_AXIS, pitchRads);
  this.offsetCamera.rotateOnAxis(this.ROLL_AXIS, rollRads);

  this.scene = new THREE.Scene();
  this.mesh = this.getMesh();

  this.scene.add(this.mesh);
  this.scene.add(this.camera);

  this.renderer = new THREE.WebGLRenderer();
  this.renderer.setPixelRatio(window.devicePixelRatio);
  this.renderer.setSize(window.innerWidth, window.innerHeight);
}

PanoClient.prototype.update = function (nowMsec) {
  this.lastTimeMsec  = this.lastTimeMsec || nowMsec-1000/60
  var deltaMsec = Math.min(200, nowMsec - this.lastTimeMsec)
  this.lastTimeMsec  = nowMsec

  this.scene.add(this.getMesh());
  if (this.videoTexture != null) {
    this.setPlaybackRate();
    this.videoTexture.update(deltaMsec / 1000, nowMsec / 1000);
  }
  this.renderer.render(this.scene, this.offsetCamera);
};

PanoClient.prototype.handleState = function(msg) {
  // ApplicationState.VISIBLE == "visible", or 3... hopefully that lasts
  this.state = (msg.state == "VISIBLE" || msg.state == 3);
  //console.log("got state: " + msg.state);
  if (this.state) {
    this.playVideo();
  } else {
    this.pauseVideo();
  }
};

PanoClient.prototype.playVideo = function() {
  if (this.videoTexture)
    this.videoTexture.video.play();
};

PanoClient.prototype.pauseVideo = function() {
  if (this.videoTexture)
    this.videoTexture.video.pause();
};

PanoClient.prototype.sendTime = function() {
  var msg = new ROSLIB.Message({
    data: this.videoTexture.video.currentTime
  });
  this.timeTopic.publish(msg);
};


PanoClient.prototype.setPlaybackRate = function() {
  /* just in case no messages have been received */
  if (!this.videoTexture)
    return;
  var currentPosition = this.videoTexture.video.currentTime;
  this.leaderPosition = this.leaderPosition || currentPosition;
  if (this.isLeader) {
    this.sendTime();
    //sendPlay();
    return;
  }
  var diff = currentPosition - this.leaderPosition;
  if (Math.abs(diff) > this.HARD_SYNC_DIFF) {
    this.videoTexture.video.playbackRate = 1;
    this.videoTexture.video.currentTime = this.leaderPosition;
    console.log("seek to: " + this.leaderPosition);
  } else {
    var rate = clamp(1 - diff, this.MIN_PLAYBACKRATE, this.MAX_PLAYBACKRATE);
    this.videoTexture.video.playbackRate = rate;
  }
};

PanoClient.prototype.windowResize = function() {
  this.camera.aspect = window.innerWidth / window.innerHeight;
  this.camera.updateProjectionMatrix();

  this.renderer.setSize(window.innerWidth, window.innerHeight);
};

PanoClient.prototype.getDomElement = function() {
  return this.renderer.domElement;
};


PanoClient.prototype.needsNewMesh = function() {
  return this.mesh == null || this.last_pano_url !== this.pano_url;
};

PanoClient.prototype.destroyVideoTexture = function() {
  if (this.videoTexture != null)
    this.videoTexture.destroy();
};

PanoClient.prototype.getMaterial = function() {
  var material;
  if (this.isMovie())
    material = this.handleMovie();
  else
    material = this.handleImage();

  return material;
};

PanoClient.prototype.isMovie = function() {
  return this.pano_url.search(/.mp4/) != -1;
};

PanoClient.prototype.handleMovie = function() {
  var material;
  this.videoTexture = new THREEx.VideoTexture(this.pano_url);
  material = this.videoTexture.texture;
  this.videoTexture.video.loop = false;
  this.videoTexture.video.setAttribute('crossorigin', 'anonymous');
  if (!this.isLeader)
    this.videoTexture.video.muted = true;
  if (this.state)
    this.playVideo();
  else
    this.pauseVideo()

  return material;
}

PanoClient.prototype.handleImage = function() {
  this.videoTexture = null;
  return THREE.ImageUtils.loadTexture(this.pano_url);
};

PanoClient.prototype.getMesh = function() {
  if (!this.needsNewMesh())
    return this.mesh;
  if (this.mesh != null)
    this.scene.remove(this.mesh);

  this.last_pano_url = this.pano_url;
  this.destroyVideoTexture();

  this.mesh = new THREE.Mesh(
      new THREE.SphereGeometry(100, 32, 32),
      new THREE.MeshBasicMaterial({
        map: this.getMaterial()
      })
  );
  this.mesh.scale.x = -1;
  return this.mesh;
}

PanoClient.prototype.setTarget = function(xTwist, yTwist, zTwist) {
  if (this.mesh == null)
    return;
  var phi = toRad(90 - xTwist);
  var theta = toRad(zTwist);

  this.camera.target.x = 500 * Math.sin(phi) * Math.cos(theta);
  this.camera.target.y = 500 * Math.cos(phi);
  this.camera.target.z = 500 * Math.sin(phi) * Math.sin(theta);

  this.camera.lookAt(this.camera.target);
};

function toRad(deg) {
  return THREE.Math.degToRad(deg);
}

function clamp(val, low, high) {
  return THREE.Math.clamp(val, low, high);
}
