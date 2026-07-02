(function() {

// Show/hide requests are sent to the offscreen document (see onboard.js),
// which owns the rosbridge connection and publishes /lg_onboard/visibility.
function showOnboard() {
  chrome.runtime.sendMessage({onboard: 'show'});
}

function hideOnboard() {
  chrome.runtime.sendMessage({onboard: 'hide'});
}

// Adds callbacks to the Maps search field.
// onclick/touchstart - shows keyboard
// onblur             - hides keyboard
function addCallbacks() {

  // for youtube, different elemnt ID, we'd need '#masthead-search-term'
  // and "*://www.youtube.com/*" in the manifest matches
  var searchbox = document.querySelector('#searchboxinput');

  if (document.readyState !== 'complete' || !searchbox) {
    setTimeout(addCallbacks, 100);
  } else {
    searchbox.addEventListener('click', showOnboard);
    searchbox.addEventListener('touchstart', showOnboard);
    searchbox.addEventListener('blur', hideOnboard);
  }
}

addCallbacks();

})();
