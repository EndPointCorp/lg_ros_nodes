/**
 *
 * Street View POV library
 *
 */

function wrap(val, low, high) {
  if (val > high) {
    val = val - (high - low);
  }
  if (val < low) {
    val = val + (high - low);
  }
  return val;
}

function toRadians(degrees) {
  return degrees * Math.PI / 180;
}

function toDegrees(radians) {
  return radians * 180 / Math.PI;
}

this.parseMatrix = function(_str)
{
  return _str.replace(/^matrix(3d)?\((.*)\)$/,'$2').split(/, /);
};

function transformHTR(htr, radianOffset) {
  var transform = M33.headingTiltRollToLocalOrientationMatrix( htr );
  transform[0] = V3.rotate(transform[0], transform[2], -radianOffset);
  transform[1] = V3.rotate(transform[1], transform[2], -radianOffset);
  var finalHTR = M33.localOrientationMatrixToHeadingTiltRoll( transform );
  return finalHTR;
}

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2

