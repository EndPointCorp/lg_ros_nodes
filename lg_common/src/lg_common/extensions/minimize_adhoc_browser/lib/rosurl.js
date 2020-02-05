
function createRosUrl(params) {
    
    function isTrue(val) {
        return val === '1' || val === 1 || val === true
            || ('' + val).toLowerCase() === 'true';
    }

    if (params['rosbridge_secure'] === undefined) {
        // Set the default value
        var protocol = 'wss://';
    }
    else {
        var protocol = (isTrue(params['rosbridge_secure']) ? 'wss://' : 'ws://');
    }
    return protocol
        + (params['rosbridge_host'] || 'localhost' ) + ':'
        + (params['rosbridge_port'] || '9090');
}

function loadGETParams(url, callback) {

    function parseUrl(url) {
        var parser = document.createElement('a');
        parser.href = url;
        return getQueryParams(parser.search);
    }
    
    function getQueryParams(qs) {
        qs = qs.split('+').join(' ');
        var params = {},
            tokens,
            re = /[?&]?([^=]+)=([^&]*)/g;
        while (tokens = re.exec(qs)) {
            params[decodeURIComponent(tokens[1])] = decodeURIComponent(tokens[2]);
        }
        return params;
    }

    var params = parseUrl(url);
    if (params['ros_instance_name'] !== undefined) {
      callback(params);
      return;
    }
    else {
        // woops, looks like google maps
        chrome.history.search({text: '', maxResults: 10}, function(histItems) {
            for (var i = 0; i < histItems.length; i++) {
                var histItem = histItems[i];
                if (histItem.url && histItem.url.indexOf('ros_instance_name') >= 0) {
                    console.log("Red from history", histItem.url);
                    var params = parseUrl(histItem.url);
                    callback(params);
                    return;
                }
            }
            console.log('ERROR: Failed to load ros_instance_name and rosbridge parameters');
        });
    }
}