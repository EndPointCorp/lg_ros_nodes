var readyTopic = null;
var ros = null;
var sentId = null;

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

function sendMsg(ros_window_name) {
    readyTopic.publish({'data': ros_window_name});
    console.log("Sent " + ros_window_name);
}

function init(params) {
    if (!readyTopic) {
        // Rosbridge url
        var rosUrl = (params['rosbridge_secure'] == 0 ? 'wss://' : 'ws://')
        + (params['rosbridge_host'] || 'localhost' ) + ':'
        + (params['rosbridge_port'] || '9090');

        ros = new ROSLIB.Ros({
            url : rosUrl
        });

        readyTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/director/window/ready',
            messageType: 'std_msgs/String',
            throttle_rate: 33
        });

        readyTopic.advertise();
    }
}

function parseUrl(url) {
    var parser = document.createElement('a');
    parser.href = url;
    return getQueryParams(parser.search);
}

// Listen for DOM_LOADED message from host message
// if page was loaded before extension.
chrome.runtime.onMessage.addListener(
    function(msg, sender, sendResponse) {
        if (msg.type == "DOM_LOADED") {
            // Active page loaded
            chrome.tabs.query({active: true}, function(tabs) {
                if (tabs.length == 0) {
                    return;
                }
                var params = parseUrl(tabs[0].url);
                var instanceName = params['ros_instance_name'];
                if (instanceName !== undefined) {
                    // reentrant safe
                    init(params);
                    // this path only for default dom loaded event
                    if(!params['use_app_event']) {
                        // Do not sent msg, if it's already been sent
                        if (sentId != instanceName) {
                            sendMsg(instanceName);
                            sentId = instanceName;
                        }
                        sendResponse({ack: sentId == instanceName});
                    }
                }
            });
        }
    }
);

chrome.webNavigation.onDOMContentLoaded.addListener(function(data) {
    if (data && data.url && data.url.indexOf('?')) {
        var params = parseUrl(data.url);
        var instanceName = params['ros_instance_name'];
        if (instanceName !== undefined) {
            init(params);
            if(params['use_app_event']) {
                chrome.runtime.onMessage.addListener(
                    function(msg, sender, sendResponse) {
                        if (msg.type == "DIRECTOR_WINDOW_READY") {
                            if (sentId != instanceName) {
                                sendMsg(instanceName);
                                sentId = instanceName;
                            }
                            sendResponse({ack: true});
                        }
                    }
                );
            }
            else {
                sendMsg(instanceName);
            }
        }
    }
});
