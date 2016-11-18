console.log("block clicks loaded...");
var xmlHttp = null;

xmlHttp = new XMLHttpRequest();
xmlHttp.open( "GET", chrome.extension.getURL ("block_div.html"), false );
xmlHttp.send( null );

var inject  = document.createElement("div");
inject.innerHTML = xmlHttp.responseText
document.body.insertBefore (inject, document.body.firstChild);
