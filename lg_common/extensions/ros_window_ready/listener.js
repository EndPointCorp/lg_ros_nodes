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

function createRosUrl(params) {
    return (params['rosbridge_secure'] == 0 ? 'wss://' : 'ws://')
        + (params['rosbridge_host'] || 'localhost' ) + ':'
        + (params['rosbridge_port'] || '9090');
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
           this.waiters[index](this.flags, flag);
        }
    }
};

State.prototype.addWaiter = function(key, waiter) {
    if (this.waiters === undefined) {
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
    this.chromeCallbacks = [];
    this.domLoadedWaiter = function(flags) {
        // Everything is ready
        if (flags.rosReady && flags.urlParsed
            // And one we auqired dom loaded one way or another
            && (flags.domLoadedMsg || flags.domLoadedEvnt)) {

            if(!extension.use_app_event) {
                extension.sendMsg();
                for (var i = 0; i < extension.chromeCallbacks.length; i++) {
                    extension.chromeCallbacks[i]({ack: true});
                }
            }
        }
    };


}

WindowReadyExt.prototype.initRos = function() {
    console.log("Starting initialization of callback extension");
    if (this.readyTopic) {
        console.log("Ros already initialized");
    }

    var extension = this;

    console.log("This is rosUrl: " + this.rosUrl);

    this.ros = new ROSLIB.Ros();

    this.ros.on('error', function(error) {
        extension.onRosError.apply(extension, [error]);
    });

    this.ros.on('connection', function() {
        this.state.update('rosReady');
        console.log('Connection made!');
    });

    console.log("Connecting ros object");
    this.ros.connect(this.rosUrl);

    this.initTopics();

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

WindowReadyExt.prototype.onRosError = function() {
    console.log( error );
};

WindowReadyExt.prototype.sendMsg = function() {
    if(this.sentIds[this.ros_window_name]) {
      console.log("Msg for window" + this.ros_window_name + " already been sent");
      return;
    }

    console.log("Going to send " + this.ros_window_name);
    this.readyTopic.publish({'data': this.ros_window_name});
    console.log("Sent " + this.ros_window_name);
    console.log("Unadvertising on the topic");
    this.readyTopic.unadvertise();
    console.log("Unadvertised on the topic");

    this.sentIds[this.ros_window_name] = true;
};

WindowReadyExt.prototype.attachListeners = function() {
    var extension = this;

    // Listen for DOM_LOADED message from host message
    // if page was loaded before extension.
    chrome.runtime.onMessage.addListener(
        function(msg, sender, sendResponse) {
            if (msg.type == "DOM_LOADED") {
                return extension.domLoadedMsg(sender, sendResponse);
            }
            if (msg.type == "DIRECTOR_WINDOW_READY") {
                return extension.directorWindowMsg(sender, sendResponse);
            }
        }
    );

    chrome.webNavigation.onDOMContentLoaded.addListener(function(data) {
        if (data && data.url && data.url.indexOf('?')) {
            var params = parseUrl(data.url);
            extension.applyUrlParams.apply(extension, [params]);
            extension.domLoadedEvnt.apply(extension, []);
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
        this.ros_window_name = ros_window_name;
        this.state.setFlag('urlParsed');
    }
};

// Dom loaded system message
WindowReadyExt.prototype.domLoadedEvnt = function() {
    this.state.addWaiter('sendMsg', this.getSendMsgwaiter());
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


WindowReadyExt.prototype.getSendMsgwaiter = function(sendResponse) {
    if(sendResponse) {
        this.chromeCallbacks.push(sendResponse);
    }
    return this.getSendMsgwaiter;
};

// Dom loaded message from hosted page
WindowReadyExt.prototype.domLoadedMsg = function(sender, sendResponse) {
    var extension = this;

    // Add actual callback which will send a message
    this.state.addWaiter('sendMsg', this.getSendMsgwaiter(sendResponse));

    // Get url parameters from
    if(sender.tabs && sender.tabs.Tab) {
        var params = parseUrl(sender.tabs.Tab.url);
        extension.applyUrlParams(params);
        extension.state.setFlag('domLoadedMsg');
    }
    else {
        chrome.tabs.query({active: true}, function(tabs) {
            if (tabs.length == 0) {
                console.log("Can't aquire url");
                return false;
            }
            var params = parseUrl(tabs[0].url);
            this.applyUrlParams(params);
            extension.state.setFlag('domLoadedMsg');
        );
    }

    // Needed to use sendResponse callback asynchronusly
    return true;
};

var INSTANCE = new WindowReadyExt();
