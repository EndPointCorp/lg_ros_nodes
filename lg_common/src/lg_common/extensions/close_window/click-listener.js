console.log("Start closeme script");
console.log("img", document.getElementsByTagName("IMG")[0]);
document.getElementsByTagName("IMG")[0].onclick = function() {
    chrome.runtime.sendMessage({type: "CLOSE"});
}
