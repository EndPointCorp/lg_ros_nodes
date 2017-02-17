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
  /*
  if (panoData.Data && panoData.Data.attribution_name) {
    attribs.push(panoData.Data.attribution_name);
  }
  if (panoData.Data && panoData.Data.copyright) {
    attribs.push(panoData.Data.copyright);
  }
  */
  if (panoData.location) {
    if (panoData.location.description) {
      attribs.push(panoData.location.description);
    }
    if (panoData.location.region && panoData.location.country) {
      attribs.push(panoData.location.region + ', ' + panoData.location.country);
    }
    else if (panoData.location.region) {
      attribs.push(panoData.location.region);
    }
    else if (panoData.location.country) {
      attribs.push(panoData.location.country);
    }
  }
  /*
  if (panoData.Data && panoData.Data.image_date) {
    attribs.push('Image Date: ' + panoData.Data.image_date);
  }
  */

  if (attribs.length == 0) {
    console.log('clearing');
    this.clear();
    return;
  }

  var attribHtml = attribs.join('<br />');

  console.log('setting', attribHtml);
  this.el.innerHTML = attribHtml;
};
