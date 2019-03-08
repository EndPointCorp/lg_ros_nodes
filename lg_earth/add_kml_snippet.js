let addedKMLTopic = new ROSLIB.Topic({
    ros: ros,
    name: 'lg_earth/added_kml',
    messageType: 'lg_common/StringArray'
});

addedKMLTopic.subscribe(function(msg){
    msg.strings // ids of added kmls
});