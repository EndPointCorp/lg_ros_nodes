function getParameterByName(name, type, default_) {
  name = name.replace(/[\[]/, '\\[').replace(/[\]]/, '\\]');
  var regex = new RegExp('[\\?&]' + name + '=([^&#]*)'),
                         results = regex.exec(location.search);
  return (results === null ? default_ : type(
          decodeURIComponent(results[1].replace(/\+/g, ' '))));
}

function isTrue(val) {
  return val === '1' ||
    val === 1 ||
    val === true ||
    ('' + val).toLowerCase() === 'true';
}

const janusHost = getParameterByName("janusHost", String, null);
const janusPort = getParameterByName("janusPort", Number, null);
const janusSecret = getParameterByName("janusSecret", String, null);
const streamID = getParameterByName("streamID", Number, null);

const janusUrl = `ws://${janusHost}:${janusPort}/`;


let janus;
let streaming;
let remoteMedia;
const video = document.createElement("video");
video.muted = true;
video.autoplay = true;
document.body.appendChild(video);

function attachStreaming() {
  console.log("attaching streaming plugin");
  janus.attach({
    plugin: "janus.plugin.streaming",
    success: (plugin) => {
      streaming = plugin;
      getStreams();
    },
    onmessage: (msg, jsep) => {
      if (jsep !== undefined && jsep !== null) {
        streaming.createAnswer({
          jsep: jsep,
          media: {audioSend: false, videoSend: false},
          success: (ourJsep) => {
            const body = {request: "start"};
            streaming.send({message: body, jsep: ourJsep});
          },
          error: console.error,
        });
      }
    },
    onremotestream: (stream) => {
      console.log("got remote stream:", stream);
      remoteMedia = stream;
      video.srcObject = stream;
    },
  });
}

function getStreams() {
  console.log("getting stream list");
  const body = {request: "list"};
  streaming.send({
    message: body,
    success: (resp) => {
      startStream(streamID);
    },
    error: console.error,
  });
}

function startStream(id) {
  console.log("starting stream", id);
  const body = {request: "watch", id: id};
  streaming.send({
    message: body,
    error: console.error,
  });
}

function init() {
  Janus.init({
    debug: false,
    dependencies: Janus.useDefaultDependencies(),
    callback: () => {
      console.log("Janus initialized");
      janus = new Janus({
        apisecret: janusSecret,
        server: janusUrl,
        success: () => {
          attachStreaming();
        },
        error: console.error,
      });
    },
  });
}

init();

