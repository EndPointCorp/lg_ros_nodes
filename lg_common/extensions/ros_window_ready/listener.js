var readyTopic = null;
var ros = null;
var sentId = {};

var requirements = {
  rosReady: false,
  domLoadedMsg: false,
  domLoadedEvnt: false,
  customReadyMsg: false
};

var waiters = [];

function update(key) {
  requirements[key] = true;
  for (var i = 0; i < waiters.length; i++){
    waiters[i]();
  }
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

function sendMsg(ros_window_name) {
    if(sentId[ros_window_name]) {
      console.log("Msg for window" + ros_window_name + " already been sent");
      return;
    }

    console.log("Going to send " + ros_window_name);
    readyTopic.publish({'data': ros_window_name});
    console.log("Sent " + ros_window_name);
    console.log("Unadvertising on the topic");
    readyTopic.unadvertise();
    console.log("Unadvertised on the topic");

    sentId[ros_window_name] = true;
}

function init(params) {
    if (!readyTopic) {
        console.log("Starting initialization of callback extension");
        // Rosbridge url
        var rosUrl = (params['rosbridge_secure'] == 0 ? 'wss://' : 'ws://')
        + (params['rosbridge_host'] || 'localhost' ) + ':'
        + (params['rosbridge_port'] || '9090');
        console.log("This is rosUrl: " + rosUrl);

        console.log("Creating ros object");
        ros = new ROSLIB.Ros();
        console.log("Created ros object");

        console.log("Registering ros callback for errors");
        ros.on('error', function(error) {

        console.log( error ); });
        ros.on('connection',
            function() {
              update('rosReady');
              console.log('Connection made!');
            });

        console.log("Registered ros callback for errors");


        console.log("Connecting ros object");
        ros.connect(rosUrl);
        console.log("Connected ros object");

        console.log("Executing callOnConnection()");
        ros.callOnConnection();
        console.log("Executed callOnConnection()");

        console.log("Initializing /director/window/ready topic");
        readyTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/director/window/ready',
            messageType: 'std_msgs/String',
            throttle_rate: 33
        });
        console.log("Initialized /director/window/ready topic");
        console.log("Advertising on the topic /director/window/ready");
        readyTopic.advertise();
        console.log("Advertised on the topic /director/window/ready");
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
                        waiters.push(function(){
                          if(requirements.rosReady) {
                            sendMsg(instanceName);
                            sendResponse({ack: true});
                          }
                        });
                        update('domLoadedMsg');
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
                            waiters.push(function(){
                                if(requirements.rosReady) {
                                    console.log('Got DIRECTOR_WINDOW_READY message');
                                    sendMsg(instanceName);
                                    sendResponse({ack: true});
                                }
                            });
                            update('customReadyMsg');
                        }
                    }
                );
            }
            else {
              waiters.push(function(){
                if(requirements.rosReady) {
                    sendMsg(instanceName);
                    console.log('Dom loaded, message sent');
                }
              });
              update('domLoadedEvnt');
            }
        }
    }
});
