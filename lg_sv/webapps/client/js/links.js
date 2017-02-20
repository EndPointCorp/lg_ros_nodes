/**
 * Street View links visualization.
 * @constructor
 * @param {THREE.Camera} camera
 */
function Links(camera, scene) {
  this.camera = camera;
  this.scene = scene;

  this.linkGroup = new THREE.Object3D();
  this.scene.add(this.linkGroup);
  this.linkGroup.position.z = -250;

  this.sphereSize = 20;

  // debug sphere for link positioning
  /*
  var sphereGeometry = new THREE.SphereGeometry(this.sphereSize, 16, 16);
  var sphereMaterial = new THREE.MeshBasicMaterial({
    wireframe: true,
    color: 0x00FF00
  });
  var sphereMesh = new THREE.Mesh(sphereGeometry, sphereMaterial);
  this.linkGroup.add(sphereMesh);
  */

  var light = new THREE.PointLight(0xffffff, 0.60, 0);
  this.linkGroup.add(light);
  light.position.y = 256;

  var ambientLight = new THREE.AmbientLight(0x303030);
  this.linkGroup.add(ambientLight);

  this.linkAnchors = [];
  this.metadata = null;
}

/**
 * Handle a pov change.
 * @param {Number} heading in degrees
 * @param {Number} tilt in degrees
 * @param {Number} roll in degrees
 * @param {Number} hFov horizontal FOV in degrees
 */
Links.prototype.handleView = function(heading, tilt, roll, hFov) {
  // Bypass when there is no data.
  if (!this.metadata) {
    return;
  }

  this.linkGroup.position.y = -this.camera.fov * 1.5;
  this.linkGroup.rotation.x = -THREE.Math.degToRad(Math.min(tilt, -8) * 0.75);
  this.linkGroup.rotation.y = THREE.Math.degToRad(heading);

  var scale = this.camera.fov / 35;
  this.linkGroup.scale.set(scale, scale, scale);
};

Links.prototype.clear = function() {
  for (var lm in this.linkAnchors) {
    var meshes = this.linkAnchors[lm].children;
    for (var m in meshes) {
      meshes[m].geometry.dispose();
      meshes[m].material.dispose();
      this.linkAnchors[lm].remove(meshes[m]);
    }
    this.linkGroup.remove(this.linkAnchors[lm]);
  }
  this.linkAnchors = [];
};

Links.prototype.update = function(panoData) {
  this.clear();
  this.metadata = panoData;

  var linkHeadings = [];
  for (var link in panoData["links"]) {
    linkHeadings.push(panoData["links"][link].heading);
  }

  for (n in linkHeadings) {
    var linkHdg = linkHeadings[n];
    var linkZ = Math.cos(THREE.Math.degToRad(-linkHdg)) * this.sphereSize;
    var linkX = Math.sin(THREE.Math.degToRad(-linkHdg)) * this.sphereSize;
    var linkMaterial = new THREE.MeshPhongMaterial({
      color: 0xffffff,
      transparent: false,
      blending: THREE.NormalBlending,
      depthTest: true,
      depthWrite: true,
      opacity: 1.0
    });
    var shape = this.drawChevron();
    var extrudeSettings = {
      amount: 0.5,
      bevelEnabled: false,
      bevelSegments: 1,
      steps: 1,
      bevelSize: 0.25,
      bevelThickness: 0.25
    };
    var geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    var linkMesh = new THREE.Mesh(geometry, linkMaterial); //, specular: 0x009900, shininess: 30, shading: THREE.FlatShading } ) );
    var linkAnchor = new THREE.Object3D();
    linkAnchor.add(linkMesh);

    var meshScale = 0.75;
    linkMesh.position.x = -8 * meshScale;
    linkMesh.position.y = -2 * meshScale;
    linkMesh.position.y = 0 * meshScale;
    linkMesh.scale.set(meshScale, meshScale, meshScale);

    /* rotate to point towards the link heading */
    linkAnchor.rotateX(THREE.Math.degToRad(90));
    linkAnchor.rotateZ(THREE.Math.degToRad(180));
    linkAnchor.rotateZ(THREE.Math.degToRad(linkHdg));

    this.linkAnchors.push(linkAnchor);
    this.linkGroup.add(linkAnchor);
    linkAnchor.position.x = linkX;
    linkAnchor.position.z = linkZ;
  }
};

Links.prototype.drawChevron = function() {
  var triangleShape = new THREE.Shape();
  triangleShape.moveTo(  8, 2 );
  triangleShape.lineTo(  0, 8 );
  triangleShape.lineTo(  2, 8 );
  triangleShape.lineTo(  8, 4 );

  triangleShape.lineTo( 14, 8 );
  triangleShape.lineTo( 16, 8 );
  triangleShape.lineTo(  8, 2 );
  return triangleShape;
};
