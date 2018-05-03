chrome.runtime.onMessage.addListener(
    function(msg, sender, sendResponse) {
        if (msg.type == "CLOSE") {
            chrome.windows.getCurrent(function(w) {
                chrome.windows.remove(w.id);
            });
        }
    }
);