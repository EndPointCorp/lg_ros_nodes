class PanoVideoScene {
  constructor(video, hFov, width, height) {
    this.video = video;
    this.hFov = hFov;

    this.pov_ = [0, 0, 0];
    this.povNeedsUpdate_ = true;

    this.scene_ = new THREE.Scene();

    let aspect = width / height;
    let near = 0.1;
    let far = 2000;
    let fov = this.getVerticalFov(width, height);
    this.camera_ = new THREE.PerspectiveCamera(fov, aspect, near, far);
    this.camera_.target = new THREE.Vector3(0, 0, 0);
    this.scene_.add(this.camera_);

    let texture = new THREE.VideoTexture(
      this.video,
      THREE.UVMapping,
      THREE.ClampToEdgeWrapping,
      THREE.ClampToEdgeWrapping,
      THREE.LinearFilter,
      THREE.LinearFilter
    );
    this.material_ = new THREE.MeshBasicMaterial({
      map: texture,
      side: THREE.DoubleSide
    });

    this.projectionTypes_ = {
      'cube': this.getCubeMesh_.bind(this),
      'equirectangular': this.getEquirectangularMesh_.bind(this)
    };
  }

  setProjection(projectionOpts) {
    if (!projectionOpts.hasOwnProperty('type')) {
      throw 'projectionOpts is missing "type"';
    }

    let projectionType = projectionOpts['type'];

    if (!this.projectionTypes_.hasOwnProperty(projectionType)) {
      throw 'Unsupported panorama projection: ' + projectionType;
    }

    if (this.mesh_) {
      this.scene_.remove(this.mesh_);
      this.mesh_.geometry.dispose();
    }

    this.mesh_ = this.projectionTypes_[projectionType](projectionOpts);
    this.scene_.add(this.mesh_);

    /*
    let wireframe = new THREE.Mesh(this.mesh_.geometry, new THREE.MeshBasicMaterial({
      color: 0xff0000,
      wireframe: true
    }));
    this.scene_.add(wireframe);
    */
  }

  getEquirectangularMesh_(projectionOpts) {
    let sphereGeometry = new THREE.SphereGeometry(1000, 64, 64);
    sphereGeometry.scale(-1, 1, 1);
    sphereGeometry.rotateY(Math.PI);
    let sphere = new THREE.Mesh(sphereGeometry, this.material_);
    return sphere;
  }

  getCubeMesh_(projectionOpts) {
    let expandCoef = projectionOpts['expandCoef'] || 1.0;
    let cubeGeometry = new THREE.BoxGeometry(1000, 1000, 1000);

    let front = [
      new THREE.Vector2(0.0, 2 / 3),
      new THREE.Vector2(0.5, 2 / 3),
      new THREE.Vector2(0.5, 1 / 3),
      new THREE.Vector2(0.0, 1 / 3)
    ];
    let back = [
      new THREE.Vector2(0.5, 1 / 3),
      new THREE.Vector2(0.5, 2 / 3),
      new THREE.Vector2(1.0, 2 / 3),
      new THREE.Vector2(1.0, 1 / 3)
    ];
    let up = [
      new THREE.Vector2(0.5, 1 / 3),
      new THREE.Vector2(1.0, 1 / 3),
      new THREE.Vector2(1.0, 0.0),
      new THREE.Vector2(0.5, 0.0)
    ];
    let down = [
      new THREE.Vector2(1.0, 2 / 3),
      new THREE.Vector2(0.5, 2 / 3),
      new THREE.Vector2(0.5, 1.0),
      new THREE.Vector2(1.0, 1.0)
    ];
    let left = [
      new THREE.Vector2(0.0, 1.0),
      new THREE.Vector2(0.5, 1.0),
      new THREE.Vector2(0.5, 2 / 3),
      new THREE.Vector2(0.0, 2 / 3)
    ];
    let right = [
      new THREE.Vector2(0.0, 1 / 3),
      new THREE.Vector2(0.5, 1 / 3),
      new THREE.Vector2(0.5, 0.0),
      new THREE.Vector2(0.0, 0.0)
    ];

    // Shrink UV's to compensate transform360 with expand_coef
    let shrinkRatio = (expandCoef - 1.0) / 2;
    function shrinkUvs(uvs, ratio) {
      let minX = Math.min(uvs[0].x, uvs[1].x, uvs[2].x, uvs[3].x);
      let maxX = Math.max(uvs[0].x, uvs[1].x, uvs[2].x, uvs[3].x);
      let minY = Math.min(uvs[0].y, uvs[1].y, uvs[2].y, uvs[3].y);
      let maxY = Math.max(uvs[0].y, uvs[1].y, uvs[2].y, uvs[3].y);
      var w = maxX - minX;
      var h = maxY - minY;
      var l = minX + w * ratio;
      var r = maxX - w * ratio;
      var d = minY + h * ratio;
      var u = maxY - h * ratio;
      for (var uv of uvs) {
        if (uv.x === minX) {
          uv.x = l;
        } else if (uv.x === maxX) {
          uv.x = r;
        }
        if (uv.y === minY) {
          uv.y = d;
        } else if (uv.y === maxY) {
          uv.y = u;
        }
      }
    }
    shrinkUvs(front, shrinkRatio);
    shrinkUvs(back, shrinkRatio);
    shrinkUvs(up, shrinkRatio);
    shrinkUvs(down, shrinkRatio);
    shrinkUvs(left, shrinkRatio);
    shrinkUvs(right, shrinkRatio);

    cubeGeometry.faceVertexUvs[0][0] = [front[0], front[1], front[3]];
    cubeGeometry.faceVertexUvs[0][1] = [front[1], front[2], front[3]];
    cubeGeometry.faceVertexUvs[0][2] = [back[0], back[1], back[3]];
    cubeGeometry.faceVertexUvs[0][3] = [back[1], back[2], back[3]];
    cubeGeometry.faceVertexUvs[0][4] = [up[0], up[1], up[3]];
    cubeGeometry.faceVertexUvs[0][5] = [up[1], up[2], up[3]];
    cubeGeometry.faceVertexUvs[0][6] = [down[0], down[1], down[3]];
    cubeGeometry.faceVertexUvs[0][7] = [down[1], down[2], down[3]];
    cubeGeometry.faceVertexUvs[0][8] = [left[0], left[1], left[3]];
    cubeGeometry.faceVertexUvs[0][9] = [left[1], left[2], left[3]];
    cubeGeometry.faceVertexUvs[0][10] = [right[0], right[1], right[3]];
    cubeGeometry.faceVertexUvs[0][11] = [right[1], right[2], right[3]];
    cubeGeometry.uvsNeedUpdate = true;

    let cube = new THREE.Mesh(cubeGeometry, this.material_);

    // Un-rotate transform360 with yaw=-45 pitch=-45
    cube.rotateY(Math.PI / 4);
    cube.rotateX(Math.PI);
    cube.rotateZ(Math.PI / 4);
    return cube;
  }

  handleResize(width, height) {
    let fov = this.getVerticalFov(width, height);
    let aspect = width / height;

    this.camera_.aspect = aspect;
    this.camera_.fov = fov;
    this.camera_.updateProjectionMatrix();
  }

  getVerticalFov(width, height) {
    let hFovRad = THREE.Math.degToRad(this.hFov);
    let vFovRad = 2 * Math.atan(Math.tan(hFovRad / 2) * (height / width));

    return THREE.Math.radToDeg(vFovRad);
  }

  setPov(heading, tilt, roll) {
    this.pov_[0] = heading;
    this.pov_[1] = tilt;
    this.pov_[2] = roll;
    this.povNeedsUpdate_ = true;
  }

  updatePov_() {
    let phi = THREE.Math.degToRad(90 - this.pov_[1]);
    let theta = THREE.Math.degToRad(this.pov_[0]);
    let sp = Math.sin(phi);
    let cp = Math.cos(phi);
    let st = Math.sin(theta);
    let ct = Math.cos(theta);
    this.camera_.target.x = 500 * sp * ct;
    this.camera_.target.y = 500 * cp;
    this.camera_.target.z = 500 * sp * st;
  }

  render(renderer, yawOffset) {
    if (this.povNeedsUpdate_) {
      this.updatePov_();
      this.povNeedsUpdate_ = false;
    }
    this.camera_.lookAt(this.camera_.target);
    this.camera_.rotateOnAxis(this.camera_.up, -yawOffset);
    renderer.render(this.scene_, this.camera_);
  }
}
