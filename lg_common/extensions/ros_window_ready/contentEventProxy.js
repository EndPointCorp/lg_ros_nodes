window.addEventListener("message", function(event) {
  // We only accept messages from ourselves
  if (event.source != window)
    return;

  if (event.data.type && (event.data.type == "DIRECTOR_WINDOW_READY")) {
      chrome.runtime.sendMessage({type: "DIRECTOR_WINDOW_READY"}, function(answer) {
          if (answer.ack) {
              console.log("DIRECTOR_WINDOW_READY sent");
          }
      });
  }
}, false);
