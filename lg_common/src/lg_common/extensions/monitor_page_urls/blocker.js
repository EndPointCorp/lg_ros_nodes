(function (){

    var templates = [];
    var redirectUrl = null;
    var stats = new LgStats();

    function load(ready) {
        $.get( "file:///opt/ep/allowed_pages.json", function( data ) {
            console.log("Allowed pages: " + data);
            var conf = JSON.parse(data);
            if (conf) {
                redirectUrl = conf.redirect_to;
                ready(conf.allowed_pages);
            }
            else {
                console.log("FAILED to load allowed pages");
            }
        });
    }

    function onTemplatesLoaded(allowed_pages) {
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
/*
    function checkTabs() {
        chrome.tabs.getAllInWindow(null, function(tabs){
            for (var i = 0; i < tabs.length; i++) {
                if(tabs[i].url) {
                    if (!allowed(tabs[i].url)) {
                        chrome.tabs.update(
                            tabs[i].id,
                            {url: "http://localhost/away.html"}
                        );
                    }
                }
            }
        });
    }
*/

/*
    chrome.webRequest.onBeforeRequest.addListener(callback,
                                                  {urls: ["<all_urls>"]},
                                                  ["blocking"]);
    setInterval(checkTabs, 500);
*/
})();
