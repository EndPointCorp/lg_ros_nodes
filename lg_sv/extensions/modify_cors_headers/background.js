chrome.webRequest.onHeadersReceived.addListener(
  function(details) {
    const allowMethods = {name:"Access-Control-Allow-Methods", value:"GET, PUT, POST, DELETE, HEAD, OPTIONS, PATCH"};
    const allowOrigin = {name:"Access-Control-Allow-Origin", value:"*"};

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
