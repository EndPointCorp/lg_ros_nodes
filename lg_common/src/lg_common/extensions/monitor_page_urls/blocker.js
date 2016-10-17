(function (){

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
            var key = decodeURIComponent(tokens[1])
            var val = decodeURIComponent(tokens[2]);
            if (params[key] === undefined) {
                params[key] = val;
            }
            else if ($.isArray(params[key])) {
                params[key].push(val);
            }
            else {
                params[key] = [params[key], val];
            }
        }
        console.log("Parsed params", params);
        return params;
    }

    var templates = [];
    var redirectUrl = null;
    var stats = new LgStats();

    function setParamsAndProceed(url, callback) {
        console.log("Read conf from url: " + url);
        var conf = parseUrl(url);
        redirectUrl = conf.redirect_to || url;
        callback(conf.allowed_urls);
    }

    function load(ready) {
        // 1. get args
        chrome.tabs.query({currentWindow: true, active: true}, function(tabs){

            if(tabs.length > 0 && tabs[0].url
                && tabs[0].url.indexOf('allowed_urls') >= 0) {
                console.log("Read conf from url");
                setParamsAndProceed(tabs[0].url, ready);
                return;
            }

            // 2. history
            chrome.history.search({text: '', maxResults: 10}, function(histItems) {
                for (var i = 0; i < histItems.length; i++) {
                    var histItem = histItems[i];
                    if (histItem.url && histItem.url.indexOf('allowed_urls') >= 0) {
                        console.log("Read conf from history");
                        setParamsAndProceed(histItem.url, ready);
                        return;
                    }
                }

                // 3. config file
                $.get( "file:///opt/ep/allowed_urls.json", function( data ) {
                    var conf = JSON.parse(data);
                    if (conf) {
                        console.log("Read conf from /opt/ep/allowed_urls.json");
                        redirectUrl = conf.redirect_to;
                        ready(conf.allowed_urls);
                    }
                    else {
                        console.log("FAILED to load allowed pages");
                    }
                });
            });
        });
    }

    function onTemplatesLoaded(allowed_pages) {
        console.log("Parse allowed urls:", allowed_pages);
        // Precompile regexes
        for (var i = 0; i < allowed_pages.length; i++) {
            if (allowed_pages[i]) {
                templates.push(new RegExp(allowed_pages[i], "i"));
            }
        }
    }

    load(onTemplatesLoaded);

    function onNavigate(details) {
        // Bypass all the requests for subframes
        if (details.parentFrameId != -1) {
            return
        }
        if(!allowed(details.url)) {
            try {
                stats.sendMsg('Block page ' + details.url);
            }
            catch (e) {
                console.log(e);
            }
            console.log('Block page ' + details.url);
            chrome.tabs.update(details.tabId, {url: redirectUrl});
        }
    }

    function allowed(url) {
        if (!templates) {
            return true;
        }

        for (var i = 0; i < templates.length; i++) {
            var tmpl = templates[i];
            if (urlMatch(url, tmpl)) {
                return true;
            }
        }

        return false;
    }

    function urlMatch(url, tmpl) {
        return tmpl.test(url);
    }

    chrome.webNavigation.onBeforeNavigate.addListener(onNavigate);
    chrome.webNavigation.onCommitted.addListener(onNavigate);

})();
