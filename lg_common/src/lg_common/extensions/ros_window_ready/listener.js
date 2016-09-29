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

function isTrue(val) {
    return val === '1' || val === 1 || val === true
        || ('' + val).toLowerCase() === 'true';
}

function createRosUrl(params) {
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
    var params = parseUrl(url);
    if (params['ros_instance_name'] !== undefined) {
        callback(params);
    }
    else {
        // woops, looks like google maps
        chrome.history.search({text: '', maxResults: 10}, function(histItems) {
            for (var i = 0; i < histItems.length; i++) {
                var histItem = histItems[i];
                if (histItem.url && histItem.url.indexOf('ros_instance_name') >= 0) {
                    var params = parseUrl(histItem.url);
                    callback(params);
                }
            }
            console.log('ERROR: Failed to load ros_instance_name and rosbridge parameters');
        });
    }
}

function State() {
    this.flags = {
      rosReady: false,
      urlParsed: false,
      domLoadedMsg: false,
      domLoadedEvnt: false,
      customReadyMsg: false
    };

    this.waiters = {};
}

State.prototype.setFlag = function(key) {
    this.flags[key] = true;
    this.callWaiters(key);
};

State.prototype.callWaiters = function(flag) {
    for(var wk in this.waiters) {
        if (this.waiters.hasOwnProperty(wk)) {
           this.waiters[wk](this.flags, flag);
        }
    }
};

State.prototype.addWaiter = function(key, waiter) {
    if (this.waiters[key] === undefined) {
        this.waiters[key] = waiter;
    }
};

function WindowReadyExt() {
    this.rosUrl = null;
    this.ros_window_name = null;

    this.readyTopic = null;
    this.ros = null;
    this.sentIds = {};

    this.state = new State();

    var extension = this;
    this.state.addWaiter('initRos', function(flags) {
        // Initialize ros, after we've got url parameters
        if (flags.urlParsed && !flags.rosReady) {
            extension.initRos();
        }
    });

    // See getSendMsgwaiter
    this.domLoadedWaiter = function(flags) {
        // Everything is ready
        if (flags.rosReady && flags.urlParsed
            // And one we auqired dom loaded one way or another
            && (flags.domLoadedMsg || flags.domLoadedEvnt)) {

            if(!extension.use_app_event) {
                extension.sendMsg();
                if (extension.chromeCallback) {
                    extension.chromeCallback({ack: true});
                }
            }
        }
    };

    this.attachListeners();
}

WindowReadyExt.prototype.initRos = function() {
    if (this.readyTopic) {
        return;
    }
    console.log("Starting initialization of callback extension");

    var extension = this;

    console.log("This is rosUrl: " + this.rosUrl);

    this.ros = new ROSLIB.Ros();

    this.ros.on('error', function(error) {
        extension.onRosError.apply(extension, [error]);
    });

    this.ros.on('connection', function() {
        extension.state.setFlag.apply(extension.state, ['rosReady']);
        console.log('Connection made!');
    });

    console.log("Connecting ros object");
    this.ros.connect(this.rosUrl);

    this.ros.callOnConnection();

    console.log("Initializing /director/window/ready topic");
    this.readyTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/director/window/ready',
        messageType: 'std_msgs/String',
        throttle_rate: 33
    });
    console.log("Initialized /director/window/ready topic");

    console.log("Advertising on the topic /director/window/ready");
    this.readyTopic.advertise();
    console.log("Advertised on the topic /director/window/ready");
};

WindowReadyExt.prototype.onRosError = function(error) {
    console.log(error);
    // If you want to retry connection to ros
    // do it here. Change the url, and call
    // this.ros.connect(this.rosUrl);
    // callbacks already binded
};

WindowReadyExt.prototype.sendMsg = function() {
    if(this.sentIds[this.ros_window_name]) {
      console.log("Msg for window" + this.ros_window_name + " already been sent");
      return;
    }

    this.sentIds[this.ros_window_name] = true;

    console.log("Going to send " + this.ros_window_name);
    this.readyTopic.publish({'data': this.ros_window_name});
    console.log("Sent " + this.ros_window_name);

    this.repeatInterval = setInterval(function() {
        this.readyTopic.publish({'data': this.ros_window_name});
        console.log("Repeat window ready msg " + this.ros_window_name);
    }, 1000);
};

WindowReadyExt.prototype.attachListeners = function() {
    var extension = this;

    // Listen for DOM_LOADED message from host message
    // if page was loaded before extension.
    chrome.runtime.onMessage.addListener(
        function(msg, sender, sendResponse) {
            if (msg.type == "DOM_LOADED") {
                return extension.domLoadedMsg.apply(extension, [sender, sendResponse]);
            }
            if (msg.type == "DIRECTOR_WINDOW_READY") {
                return extension.directorWindowMsg.apply(extension, [sender, sendResponse]);
            }
        }
    );

    chrome.webNavigation.onDOMContentLoaded.addListener(function(data) {
        if (data && data.url) {
            loadGETParams(data.url, function(params) {
                extension.applyUrlParams.apply(extension, [params]);
                extension.domLoadedEvnt.apply(extension, []);
            });
        }
    });
};

// * Parse and set ros url
// * Initialize ROS
// * Set ros_instance_name
WindowReadyExt.prototype.applyUrlParams = function(params) {
    this.rosUrl = createRosUrl(params);

    // Used !! to explicitly convert to boolean
    this.use_app_event = !!params['use_app_event'];

    var instanceName = params['ros_instance_name'];
    if (instanceName !== undefined) {
        this.ros_window_name = instanceName;
        this.state.setFlag('urlParsed');
    }
};

// Dom loaded system message
WindowReadyExt.prototype.domLoadedEvnt = function() {
    this.state.addWaiter('sendMsg', this.domLoadedWaiter);
    this.state.setFlag('domLoadedEvnt');
};

// Custom message recieved
WindowReadyExt.prototype.directorWindowMsg = function(sender, sendResponse) {
    console.log('Got DIRECTOR_WINDOW_READY message');

    var extension = this;
    this.state.addWaiter('directorWindowMsg', function(flags) {
        // Initialize ros, after we've got url parameters
        if (flags.urlParsed && flags.rosReady && flags.customReadyMsg) {
            if (extension.use_app_event) {
                extension.sendMsg();
            }
            sendResponse({ack: true});
        }
    });
    this.state.setFlag('customReadyMsg');
    // Return true for chrome, to deal with sendResponse asychronosly
    return true;
};

// Dom loaded message from hosted page
WindowReadyExt.prototype.domLoadedMsg = function(sender, sendResponse) {
    var extension = this;
    this.chromeCallback = sendResponse;
    // Add actual callback which will send a message
    this.state.addWaiter('sendMsg', this.domLoadedWaiter);

    // Get url parameters from
    if(sender.url) {
        loadGETParams(sender.url, function(params){
            extension.applyUrlParams.apply(extension, [params]);
            extension.state.setFlag('domLoadedMsg');
        });
    }
    else {
        chrome.tabs.query({active: true}, function(tabs) {
            if (tabs.length == 0) {
                console.log("Can't aquire url");
                return false;
            }
            loadGETParams(tabs[0].url, function(params){
                extension.applyUrlParams(params);
                extension.state.setFlag('domLoadedMsg');
            });
        });
    }

    // Needed to use sendResponse callback asynchronusly
    return true;
};

var INSTANCE = new WindowReadyExt();
