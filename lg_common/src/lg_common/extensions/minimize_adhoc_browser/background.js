chrome.runtime.onMessage.addListener(
    function(msg, sender, sendResponse) {
        if (msg.type == "MINIMIZE") {
            loadGETParams(msg.url, sendMinimizeMsg);
        }
    }
);

function sendMinimizeMsg(params) {
    var wndid = params['ros_instance_name'];
    var rosurl = createRosUrl(params);

    var ros = new ROSLIB.Ros({url: rosurl});
    ros.on('connection', function() {
        console.log('Connection made!');

        topic = new ROSLIB.Topic({
            ros: ros,
            name: '/director/minimize',
            messageType: 'std_msgs/String',
            throttle_rate: 33,
            queue_length: 1,
        });
        topic.advertise();

        topic.publish({'data': wndid});
    });
}
