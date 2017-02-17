/**
 * Generates a fudge value for a horizontal fov.
 *
 * The values here apply to scaleFactor 2.04.
 *
 * Find and insert your own to support other scaleFactors.
 *
 * @param {Number} hFov in degrees
 * @return {Number} fudge value
 */
var getFovFudge = (function streetViewFovFudge() {
  /** FOV (x) data points. */
  var interpX = [
    12,
    15,
    20,
    25,
    30,
    46.5,
    63
  ];
  /** zoomLevel (Y) data points. */
  var interpY = [
    0.989,
    0.984,
    0.970,
    0.954,
    0.937,
    0.869,
    0.799
  ];

  /**
   * Linear interpolation for a range of data points.
   *
   * @param {Number} point
   * @param {Array} xValues sorted from low to high
   * @param {Array} yValues corresponding to xValues
   * @return {Number} interpolated value
   */
  function interpolate(point, xValues, yValues) {
    var left = 0;
    var right = xValues.length - 1;

    // Bypass min/max cases.
    if (point < xValues[left]) {
      return yValues[left];
    }
    if (point > xValues[right]) {
      return yValues[right];
    }

    // Binary search for index.
    var currentIndex;

    while (right - left !== 1) {
      currentIndex = (left + right) / 2 | 0;

      if (point >= xValues[currentIndex]) {
        left = currentIndex;
      } else {
        right = currentIndex;
      }
    }

    var leftX = xValues[left];
    var rightX = xValues[left + 1];
    var leftY = yValues[left];
    var rightY = yValues[left + 1];

    var mu = (point - leftX) / (rightX - leftX);
    var val = leftY * (1 - mu) + rightY * mu;

    return val;
  }

  return (function(hFov) {
    return interpolate(hFov, interpX, interpY);
  })
})();

