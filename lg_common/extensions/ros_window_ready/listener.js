var readyTopic = null;
var ros = null;

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
        var rosUrl = (params['rosbridge_secure'] == 0 ? 'ws://' : 'wss://')
        + (params['rosbridge_host'] || '42-b' ) + ':'
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

chrome.webNavigation.onDOMContentLoaded.addListener(function(data) {
    if (data && data.url && data.url.indexOf('?')) {
        var parser = document.createElement('a');
        parser.href = data.url;
        var params = getQueryParams(parser.search);
        var instanceName = params['ros_instance_name'];
        if (instanceName !== undefined) {
            init(params);
            if(params['ros_window_ready_evnt']) {
                chrome.runtime.onMessage.addListener(
                    function(msg, sender, sendResponse) {
                        if (msg.type == "DIRECTOR_WINDOW_READY") {
                            sendMsg(instanceName);
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
