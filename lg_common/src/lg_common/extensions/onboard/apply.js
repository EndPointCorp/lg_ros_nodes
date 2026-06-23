(function() {

var port = chrome.runtime.connect();

function showOnboard() {
  port.postMessage({show: true});
  console.log('Showing onboard with port.postMessage');
}

function hideOnboard() {
  port.postMessage({show: false});
  console.log('Hiding onboard with port.postMessage');
}

// Adds callbacks to all the input and textarea fields.
// onclick/touchstart - shows keyboard
// onblur             - hides keyboard
function addCallbacks() {

  // for youtube, different elemnt ID, we'd need '#masthead-search-term'
  // and "*://www.youtube.com/*" in the manifest matches
  var searchbox = document.querySelector('#searchboxinput');

  if (document.readyState !== 'complete' || !searchbox) {
    setTimeout(addCallbacks, 100);
    console.log('Waiting for #searchboxinput to become available');
  } else {

    searchbox.addEventListener('click', showOnboard);
    searchbox.addEventListener('touchstart', showOnboard);
    searchbox.addEventListener('blur', hideOnboard);

    console.log('#searchboxinput and become available');

  }
}

addCallbacks();
console.log('Ran addCallbacks for #searchboxinput');

})();


