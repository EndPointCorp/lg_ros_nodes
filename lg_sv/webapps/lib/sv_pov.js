/**
 *
 * Street View POV library
 *
 * @param {float} val
 * @param {float} low
 * @param {float} high
 *
 * @return {float} val
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

/**
 * Converts degrees to radians
 * @param {float} degrees
 * @return {float}
 */
function toRadians(degrees) {
  return degrees * Math.PI / 180;
}

/**
 * Converts radians to degrees
 * @param {float} radians
 * @return {float}
 */
function toDegrees(radians) {
  return radians * 180 / Math.PI;
}

/**
 * Parses a matrix element into an array
 * @param {str} _str
 * @return {str}
 */
this.parseMatrix = function(_str)
{
  return _str.replace(/^matrix(3d)?\((.*)\)$/, '$2').split(/, /);
};

/**
 * Transforms a HTR(heading, tilt, roll) into a local orientation matrix
 * and then applies the radian offset and converts back to
 * a HTR
 *
 * @param {Array} htr
 * @param {float} radianOffset
 * @return {Array} finalHTR
 */
function transformHTR(htr, radianOffset) {
  var transform = M33.headingTiltRollToLocalOrientationMatrix(htr);
  transform[0] = V3.rotate(transform[0], transform[2], -radianOffset);
  transform[1] = V3.rotate(transform[1], transform[2], -radianOffset);
  var finalHTR = M33.localOrientationMatrixToHeadingTiltRoll(transform);
  return finalHTR;
}

// # vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2

