/*
 * Onboard on-screen keyboard integration.
 *
 * Runs in the offscreen document (see offscreen.html) so that roslib.js has a
 * real DOM to work with under Manifest V3.
 *
 * Shows onboard when the Maps search field is tapped.
 * Hides onboard when tapped anywhere else, or when the spacenav is moved.
 *
 * Show/hide requests arrive over chrome.runtime messaging:
 *   - from the content script (apply.js): {onboard: 'show'|'hide'}
 *   - from the service worker (toolbar click): {onboard: 'toggle'}
 */

var onboardRos = new AlbatRos();

// This topic object is used for publishing the show and hide messages.
var onboardPublisher = new ROSLIB.Topic({
  ros: onboardRos,
  name: '/lg_onboard/visibility',
  messageType: 'std_msgs/Bool',
  throttle_rate: 33
});

onboardPublisher.advertise();

// Messages to be sent by onboardPublisher, they show and hide onboard.
var onboardShowMsg = new ROSLIB.Message({data: true});
var onboardHideMsg = new ROSLIB.Message({data: false});

// We need to hide the keyboard when spacenavigator is touched
// for that we need to listen to the spacenav/twist
// and react when there is something else than zero anywhere.
var onboardSpacenavListener = new ROSLIB.Topic({
  ros: onboardRos,
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
  throttle_rate: 33
});

onboardSpacenavListener.subscribe(function(msg) {
  if (msg.linear.x != 0 ||
      msg.linear.y != 0 ||
      msg.linear.z != 0 ||
      msg.angular.x != 0 ||
      msg.angular.y != 0 ||
      msg.angular.z != 0) {
    hideOnboard();
  }
});

var keyboardVisible = false;

function showOnboard() {
  onboardPublisher.publish(onboardShowMsg);
  keyboardVisible = true;
}

function hideOnboard() {
  onboardPublisher.publish(onboardHideMsg);
  keyboardVisible = false;
}

function toggleOnboard() {
  if (keyboardVisible) {
    hideOnboard();
  } else {
    showOnboard();
  }
}

chrome.runtime.onMessage.addListener(function(message) {
  if (!message || typeof message.onboard === 'undefined') {
    return;
  }
  if (message.onboard === 'show') {
    showOnboard();
  } else if (message.onboard === 'hide') {
    hideOnboard();
  } else if (message.onboard === 'toggle') {
    toggleOnboard();
  }
});
