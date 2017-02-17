/**
 * A module for showing information about panoramas.
 * @constructor
 * @param {HTMLElement} el - element to be updated with info
 */
function Attribution(el) {
  this.el = el;
}

/**
 * Clears the attribution text.
 */
Attribution.prototype.clear = function() {
  this.el.innerText = '';
};

/**
 * Shows attribution text for the given panorama metadata.
 * @param {object} panoData - Street View panorama metadata
 */
Attribution.prototype.handleMetadata = function(panoData) {
  var attribs = [];

  if (panoData.hasOwnProperty('copyright')) {
    attribs.push(panoData.copyright);
  }
  if (panoData.hasOwnProperty('location')) {

    if (panoData.location.hasOwnProperty('description')) {
      attribs.push(panoData.location.description);

    } else if (panoData.location.hasOwnProperty('shortDescription')) {
      attribs.push(panoData.location.shortDescription);
    }

  }
  if (panoData.hasOwnProperty('imageDate')) {
    attribs.push(panoData.imageDate);
  }

  if (attribs.length == 0) {
    this.clear();
    return;
  }

  var attribHtml = attribs.join('<br />');

  this.el.innerHTML = attribHtml;
};
