var earth = false;
var clouds = false;
chrome.webRequest.onCompleted.addListener(
  function(details) {
      if(!earth) {
          earth = true;
          console.log("Earth images loaded");
          setTimeout(callContentScript, 1000);
      }
  },
  {urls: ["https://*.google.com/rt/earth/NodeData/*"]}
);
chrome.webRequest.onCompleted.addListener(
  function(details) {
      if(!clouds) {
          clouds = true;
          console.log("Cloud images loaded");
          callContentScript();
      }
  },
  {urls: ["https://*.google.com/mw-weather/clouds-cubemap/*"]}
);
function callContentScript() {
    chrome.tabs.query({active: true, currentWindow: true}, function(tabs){
        chrome.tabs.sendMessage(
            tabs[0].id,
            {action: "send_ready_msg"},
            function(response) {
                if(response.ack) {
                    console.log("Message sent");
                }
            }
        );
    });
}
// blob:https://www.google.com/37f11e2e-ee69-4395-a8ee-140dc9431ff1
