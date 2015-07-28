// math3d.js
/*
Copyright 2008 Google Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Some Javascript math utilities.
//
// Exports V3 (3-vector utilities), M33 (3x3 matrix utilities)

// NOTE: This will be refactored in a more Object
// Oriented style, so don't get attached to this syntax!

// 3D vector functions.
V3 = {
  EARTH_RADIUS: 6378100,

  dup: function(a) {
    return [a[0], a[1], a[2]];
  },

  toString: function(a) {
    return "[" + a[0] + ", " + a[1] + ", " + a[2] + "]";
  },

  nearlyEqual: function(a, b, tolerance) {
    if (!tolerance) {
      tolerance = 1e-6;
    }
    return Math.abs(a[0] - b[0]) <= tolerance
      && Math.abs(a[1] - b[1]) <= tolerance
      && Math.abs(a[2] - b[2]) <= tolerance;
  },
  
  cross: function(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0] ];
  },

  dot: function(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  },

  add: function(a, b) {
    return [
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2]];
  },

  sub: function(a, b) {
    return [
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2]];
  },

  scale: function(a, scale) {
    return [a[0] * scale, a[1] * scale, a[2] * scale];
  },

  length: function(a) {
    return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  },

  normalize: function(a) {
    var len = V3.length(a);
    if (len <= 0) {
      return [NaN, NaN, NaN];
    }
    return V3.scale(a, 1.0 / len);
  },

  bisect: function(a, b) {
    return [(a[0] + b[0]) / 2,
            (a[1] + b[1]) / 2,
            (a[2] + b[2]) / 2];
  },

  // Returns v rotated counterclockwise about axis by radians.
  // axis should be a unit vector; otherwise you'll get weird results.
  rotate: function(v, axis, radians) {
    var vDotAxis = V3.dot(v, axis);
    var vPerpAxis = V3.sub(v, V3.scale(axis, vDotAxis));
    var vPerpPerpAxis = V3.cross(axis, vPerpAxis);
    var result = V3.add(V3.scale(axis, vDotAxis),
                        V3.add(V3.scale(vPerpAxis, Math.cos(radians)),
                               V3.scale(vPerpPerpAxis, Math.sin(radians))));
    return result;
  },

  // Takes a set of Euler angles and converts from degrees to radians.
  toRadians: function(v) {
    return [v[0] * Math.PI / 180,
            v[1] * Math.PI / 180,
            v[2] * Math.PI / 180];
  },

  // Takes a set of Euler angles and converts from radians to degrees.
  toDegrees: function(v) {
    return [v[0] * 180 / Math.PI,
            v[1] * 180 / Math.PI,
            v[2] * 180 / Math.PI];
  },

  // Input is [lat, lon, alt].  Lat & lon are in degrees, positive up
  // and east.  Alt in meters, relative to Earth's radius.
  //
  // Output is meters x,y,z.  x points out of (0,0) (just off West
  // Africa), y points out the North Pole, and z points out of (0,-90)
  // (near Ecuador).
  latLonAltToCartesian: function(vert) {
    var sinTheta = Math.sin(vert[1] * Math.PI / 180);
    var cosTheta = Math.cos(vert[1] * Math.PI / 180);
    var sinPhi = Math.sin(vert[0] * Math.PI / 180);
    var cosPhi = Math.cos(vert[0] * Math.PI / 180);

    var r = V3.EARTH_RADIUS + vert[2];
    var result = [
        r * cosTheta * cosPhi,
        r * sinPhi,
        r * -sinTheta * cosPhi ];
    return result;
  },

  // Input is meters [x, y, z].  Output is [lat, lon, alt].  Lat & lon
  // in degrees, alt in meters.
  // 
  // V3.cartesianToLatLonAlt([R, 0, 0]) ~= [0, 0, 0]
  // V3.cartesianToLatLonAlt([R/sqrt(2), R/sqrt(2), 0]) ~= [45, 0, 0]
  // V3.cartesianToLatLonAlt([R/sqrt(2), 0, R/sqrt(2)]) ~= [0, -45, 0]
  cartesianToLatLonAlt: function(a) {
    var r = V3.length(a);
    if (r <= 0) {
      return [NaN, NaN, NaN];
    }
    var alt = r - V3.EARTH_RADIUS;
    // Compute projection onto unit sphere.
    var n = V3.scale(a, 1 / r);
    var lat = Math.asin(n[1]) * 180 / Math.PI;
    if (lat > 90) {
      lat -= 180;
    }
    var lon = 0;
    if (Math.abs(lat) < 90) {
      lon = Math.atan2(n[2], n[0]) * -180 / Math.PI;
    }
    return [lat, lon, alt];
  },

  // Return the signed perpendicular distance from the point c to the line
  // defined by [a, b].
  //
  // We get the sign by determining if point is to the left of the line,
  // from the point of view of looking towards the origin through vert0.
  // I.e. is it to the left, looking at the surface of the Earth from
  // above.
  leftDistance: function(a, b, c) {
    var ab = V3.sub(b, a);
    var ac = V3.sub(c, a);
    var cross = V3.cross(ab, ac);

    var dot = V3.dot(a, cross);
    var lineLength = V3.length(ab);
    if (lineLength < 1e-6) {
      return NaN;
    }
    var perpendicularDistance = V3.length(cross) / lineLength;

    if (dot > 0) {
      return perpendicularDistance;
    } else {
      return -perpendicularDistance;
    }
  },

  // Return the distance between two cartesian 3d points, along the
  // surface of the Earth, assuming they are on the surface of the
  // Earth.  (If the inputs are not on the surface of the earth, they
  // are projected to the surface first.)
  earthDistance: function(a, b) {
    var dot = V3.dot(V3.normalize(a), V3.normalize(b));
    var angle = Math.acos(dot);
    var dist = V3.EARTH_RADIUS * angle;
    return dist;
  }
};

M33 = {
  // Conventions:
  //
  // * V3 is a 3-element array representing a column vector
  //
  // * M33 is an array of 3 column vectors, representing a 3x3 matrix
  //
  //   [ [00] [10] [20] ]
  //   [ [01] [11] [21] ]
  //   [ [02] [12] [22] ]
  
  toString: function(a) {
    return "[" + V3.toString(a[0]) + ", " + 
      V3.toString(a[1]) + ", " + V3.toString(a[2]) + "]";
  },

  nearlyEqual: function(a, b) {
    return V3.nearlyEqual(a[0], b[0])
      && V3.nearlyEqual(a[1], b[1])
      && V3.nearlyEqual(a[2], b[2]);
  },

  transpose: function(a) {
    return [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]]];
  },

  multiply: function(a, b) {
    var result = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (var i = 0; i < 3; i++) {
      for (var j = 0; j < 3; j++) {
        result[i][j] = a[0][j] * b[i][0]
                       + a[1][j] * b[i][1]
                       + a[2][j] * b[i][2];
      }
    }
    return result;
  },

  // Applies matrix a to column vector b.  (I.e. returns a * b)
  transform: function(a, b) {
    return [
        a[0][0] * b[0] + a[1][0] * b[1] + a[2][0] * b[2],
        a[0][1] * b[0] + a[1][1] * b[1] + a[2][1] * b[2],
        a[0][2] * b[0] + a[1][2] * b[1] + a[2][2] * b[2]];
  },

  // Applies the transpose of matrix a to column vector b.
  // (I.e. returns a.transpose() * b)
  transformByTranspose: function(a, b) {
    return [
        a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2],
        a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2],
        a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2]];
  },

  identity: function() {
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  },

  makeOrthonormalFrame: function(dir, up) {
    var newright = V3.normalize(V3.cross(dir, up));
    var newdir = V3.normalize(V3.cross(up, newright));
    var newup = V3.cross(newright, newdir);
    return [newright, newdir, newup];
  },

  // [heading, tilt, roll] (degrees) to [[right],[dir],[up]] (local
  // coords).  The return value transforms global direction vectors
  // into local direction vectors.  The transpose of the return value
  // transforms local direction vectors into global direction vectors.
  //
  // heading, tilt, roll are in degrees, clockwise about z,x,y axes (!?).
  // heading of 0 means pointing North
  // heading of 90 means pointing West
  //
  // [1, 0, 0] in local coords points right (East, for 0, 0, 0 htr)
  // [0, 1, 0] in local coords points ahead (North, for 0, 0, 0 htr)
  // [0, 0, 1] in local coords points up (away from center of earth)
  headingTiltRollToLocalOrientationMatrix: function(htr) {
    return M33.eulToMat(V3.toRadians(htr));
  },

  // [[right], [dir], [up]] (in local cartesian coords) to [heading, tilt, roll]
  // (in degrees)
  localOrientationMatrixToHeadingTiltRoll: function(mat) {
    var htr = M33.matToEul(mat);
    return V3.toDegrees(htr);
  },

  // Builds a local orientation matrix, to transform from local coords
  // into global coords.  "Local" means that the local x basis vector
  // points East, the y basis vector points North and the z basis
  // vector points Up (towards the sky).
  makeLocalToGlobalFrame: function(latLonAlt) {
    var vertical = V3.normalize(V3.latLonAltToCartesian(latLonAlt));
    var east = V3.normalize(V3.cross([0, 1, 0], vertical));
    var north = V3.normalize(V3.cross(vertical, east));

    return [east, north, vertical];
  },

  // See Graphics Gems IV, Chapter III.5, "Euler Angle Conversion" by
  // Ken Shoemake.
  //
  // http://vered.rose.utoronto.ca/people/spike/GEMS/GEMS.html
  eulerConfig: {
    i: 2, j: 0, k: 1,     // NOTE: KML convention is Z, X, Y!
    counterClockwise: true,
    sameAxis: false,      // third axis same as first (i == k)
    frameRelative: false  // frame-relative (vs. static)
  },

  // Construct matrix from Euler angles (in radians).
  //
  // Thanks to Ken Shoemake / Graphics Gems
  eulToMat: function(eulerAnglesIn) {
    var ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
    var config = M33.eulerConfig;
    var i = config.i;
    var j = config.j;
    var k = config.k;

    var ea = V3.dup(eulerAnglesIn);
    var m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    if (config.frameRelative) { var t = ea[0]; ea[0] = ea[2]; ea[2] = t; }
    if (!config.counterClockwise) {
      ea[0] = -ea[0]; ea[1] = -ea[1]; ea[2] = -ea[2];
    }
    ti = ea[0];	  tj = ea[1];	th = ea[2];
    ci = Math.cos(ti); cj = Math.cos(tj); ch = Math.cos(th);
    si = Math.sin(ti); sj = Math.sin(tj); sh = Math.sin(th);
    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
    if (config.sameAxis) {
      m[i][i] = cj;     m[i][j] =  sj*si;    m[i][k] =  sj*ci;
      m[j][i] = sj*sh;  m[j][j] = -cj*ss+cc; m[j][k] = -cj*cs-sc;
      m[k][i] = -sj*ch; m[k][j] =  cj*sc+cs; m[k][k] =  cj*cc-ss;
    } else {
      m[i][i] = cj*ch; m[i][j] = sj*sc-cs; m[i][k] = sj*cc+ss;
      m[j][i] = cj*sh; m[j][j] = sj*ss+cc; m[j][k] = sj*cs-sc;
      m[k][i] = -sj;   m[k][j] = cj*si;    m[k][k] = cj*ci;
    }
    return m;
  },

  // Convert matrix to Euler angles (in radians).
  //
  // Thanks to Ken Shoemake / Graphics Gems
  matToEul: function(m, config) {
    var config = M33.eulerConfig;
    var i = config.i;
    var j = config.j;
    var k = config.k;

    var FLT_EPSILON = 1e-6;
    var ea = [0, 0, 0];
    if (config.sameAxis) {
      var sy = Math.sqrt(m[i][j] * m[i][j] + m[i][k] * m[i][k]);
      if (sy > 16 * FLT_EPSILON) {
        ea[0]= Math.atan2(m[i][j], m[i][k]);
        ea[1]= Math.atan2(sy, m[i][i]);
        ea[2]= Math.atan2(m[j][i], -m[k][i]);
      } else {
        ea[0]= Math.atan2(-m[j][k], m[j][j]);
        ea[1]= Math.atan2(sy, m[i][i]);
        ea[2]= 0;
      }
    } else {
      var cy = Math.sqrt(m[i][i] * m[i][i] + m[j][i] * m[j][i]);
      if (cy > 16 * FLT_EPSILON) {
        ea[0]= Math.atan2(m[k][j], m[k][k]);
        ea[1]= Math.atan2(-m[k][i], cy);
        ea[2]= Math.atan2(m[j][i], m[i][i]);
      } else {
        ea[0]= Math.atan2(-m[j][k], m[j][j]);
        ea[1]= Math.atan2(-m[k][i], cy);
        ea[2]= 0;
      }
    }
    if (!config.counterClockwise) {
      ea[0] = -ea[0]; ea[1] = -ea[1]; ea[2] = -ea[2];
    }
    if (config.frameRelative) { var t = ea[0]; ea[0] = ea[2]; ea[2] = t; }
    return ea;
  }
};
