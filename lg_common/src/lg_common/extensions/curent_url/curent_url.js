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

function CurrentUrlExt(params) {
    this.parameters = params || {};
    this.readInterval = params.curent_url_read_interval || 500;
    this.browserKey = params.ros_instance_name;
    this.viewport = params.viewport;

    this.rosbridgeUrl = createRosUrl(this.parameters);
    this.lastURL = null;
    this.initRos();
    this.initWatch();
}

CurrentUrlExt.prototype.initWatch = function() {
    var service = this;
    service.watchInterval = setInterval(function(){
        chrome.tabs.query({currentWindow: true, active: true}, function(tabs) {
            if(tabs.length > 0 && tabs[0].url) {
                service.handleUrl.apply(service, [tabs[0].url]);
            }
        });
    }, service.readInterval);
};

CurrentUrlExt.prototype.normalizeURL = function(url) {
    return url;
};

CurrentUrlExt.prototype.handleUrl = function(curentURL) {
    var url = this.normalizeURL(curentURL);
    if (url != this.lastURL) {
        this.sendURL(url);
    }
};

CurrentUrlExt.prototype.sendURL = function(url) {
    if (this.topic) {
        this.topic.publish({
            'url': url,
            'browser_id': this.browserKey
        });
    }
    this.lastURL = url;
};

CurrentUrlExt.prototype.onRosError = function(error) {
    console.log('Ros initialization error', error);
};

CurrentUrlExt.prototype.onRosConneted = function(error) {

    var topicName = '/browser_service/' + this.viewport + '/update_url';
    this.topic = new ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType: 'std_msgs/String',
        throttle_rate: 33
    });

    this.topic.advertise();
    console.log('Topic ' + topicName + ' advertized');

    if (this.lastURL) {
        this.sendURL(this.lastURL);
    }
};

CurrentUrlExt.prototype.initRos = function(url) {
    if (this.topic) {
        return;
    }
    var extension = this;

    console.log("This is rosUrl: " + this.rosbridgeUrl);

    this.ros = new ROSLIB.Ros();

    this.ros.on('error', function(error) {
        extension.onRosError.apply(extension, [error]);
    });

    this.ros.on('connection', function() {
        extension.onRosConneted.apply(extension, []);
        console.log('Connection made!');
    });

    this.ros.connect(this.rosbridgeUrl);
    this.ros.callOnConnection();
};

getUrlArgs(function(params){
    extension = new CurrentUrlExt(params);
}, ['ros_instance_name', 'rosbridge']);
