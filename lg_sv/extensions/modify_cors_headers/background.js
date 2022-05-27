function removeHeader(headers, name) {
  for (var i = 0; i < headers.length; i++) {
    if (headers[i].name.toLowerCase() == name) {
      headers.splice(i, 1);
      break;
    }
  }
}

chrome.webRequest.onHeadersReceived.addListener(
  function(details) {
    const allowMethods = {name:"Access-Control-Allow-Methods", value:"GET, PUT, POST, DELETE, HEAD, OPTIONS, PATCH"};
    const allowOrigin = {name:"Access-Control-Allow-Origin", value:"*"};
    removeHeader(details.responseHeaders,'access-control-allow-origin');
    const responseHeaders = details.responseHeaders.concat(allowOrigin, allowMethods);
    return { responseHeaders };
  },
  // filters
  {
    urls: ["<all_urls>"],
  },
  // extraInfoSpec
  ["blocking","responseHeaders", "extraHeaders"]
);
