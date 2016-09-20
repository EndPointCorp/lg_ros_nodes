chrome.extension.onMessage.addListener(function(msg, sender, sendResponse) {
  if (msg.action == 'send_ready_msg') {
    window.postMessage({ type: "DIRECTOR_WINDOW_READY" }, "*");
    sendResponse({"ack": true});
  }
});
