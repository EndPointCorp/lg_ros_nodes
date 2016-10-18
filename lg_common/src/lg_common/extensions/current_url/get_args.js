function parseUrl(url) {
    var parser = document.createElement('a');
    parser.href = url;
    return getQueryParams(parser.search);
}

function isArray(subj) {
    return Object.prototype.toString.call( subj ) === '[object Array]';
}

function getQueryParams(qs) {
    qs = qs.split('+').join(' ');
    var params = {},
        tokens,
        re = /[?&]?([^=]+)=([^&]*)/g;
    while (tokens = re.exec(qs)) {
        var key = decodeURIComponent(tokens[1])
        var val = decodeURIComponent(tokens[2]);
        if (params[key] === undefined) {
            params[key] = val;
        }
        else if (isArray(params[key])) {
            params[key].push(val);
        }
        else {
            params[key] = [params[key], val];
        }
    }
    if (Object.keys(params).length > 0) {
        console.log("Url params parsed: ", params);
    }
    return params;
}

function requiredArgExists(url, required) {
    if(isArray(required)) {
        for (var i = 0; i < required.length; i++) {
            if (url.indexOf(required[i]) < 0) {
                return false;
            }
        }
        return true;
    }
    else {
        return url.indexOf(required) >= 0;
    }
}

function getUrlArgs(callback, required, noHistory) {

    // 1. get args
    chrome.tabs.query({currentWindow: true, active: true}, function(tabs){

        if(tabs.length > 0 && tabs[0].url && requiredArgExists(tabs[0].url, required)) {
            console.log("Read conf from url");
            callback(parseUrl(tabs[0].url));
            return;
        }

        if (!noHistory) {
            // 2. history
            chrome.history.search({text: '', maxResults: 10}, function(histItems) {
                for (var i = 0; i < histItems.length; i++) {
                    var histItem = histItems[i];
                    if (histItem.url && requiredArgExists(histItem.url, required)) {
                        console.log("Read conf from history");
                        callback(parseUrl(histItem.url));
                        return;
                    }
                }
            });
        }

        // 3. Active url anyway
        callback(parseUrl(tabs[0].url));

    });
}
