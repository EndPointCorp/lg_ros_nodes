/*
 * Onboard on-screen keyboard integration.
 * Shows onboard when tapped the search field.
 * Hides onboard when tapped anywhere else, or spacenav is moved.
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


function showOnboard() {
  //console.log('Showing Onboard keyboard');
  onboardPublisher.publish(onboardShowMsg);
}

function hideOnboard() {
  //console.log('Hiding Onboard keyboard');
  onboardPublisher.publish(onboardHideMsg);
}

chrome.runtime.onConnect.addListener(function(port) {
  port.onMessage.addListener(function(msg) {
    if (msg.show) {
      showOnboard();
    } else {
      hideOnboard();
    }
  });
});

var keyboardVisible = false;

chrome.browserAction.onClicked.addListener(function(tab) {

  if (keyboardVisible) {
    hideOnboard();
    keyboardVisible = false;
  } else {
    showOnboard();
    keyboardVisible = true;
  }
});
