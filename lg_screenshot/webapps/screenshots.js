var system = require('system');
var page = require("webpage").create();

var DEFAUL_UA = 'Mozilla/5.0 (X11; Linux x86_64) ' +
    'AppleWebKit/537.36 (KHTML, like Gecko) ' +
    'Chrome/53.0.2785.116 Safari/537.36';

var DEFAULT_WIDTH = 1800;
var DEFAULT_ZOOM = 1;
var DEFAULT_OUT = 'screenshot.png';

var args = {
    'pos': []
};
var argKey = null;
var argVal = null;
for (var i = 1; i < system.args.length; i++) {
    if(system.args[i].indexOf('--') == 0) {
        if (argVal && argKey) {
            args[argKey] = argVal;
        }
        argKey = system.args[i].substring(2);
        argVal = null;
    }
    else if (argKey) {
        if (argVal) {
            if (Array.isArray(argVal)) {
                argVal.push(system.args[i]);
            }
            else {
                argVal = [argVal, system.args[i]];
            }
        }
        else {
            argVal = system.args[i];
        }
    }
    else {
        args.pos.push(system.args[i]);
    }
}
if (argKey) {
    args[argKey] = argVal;
}

if (args.zoom) {
    console.log('Use zoom', args.zoom);
}
var zoom = args.zoom || DEFAULT_ZOOM
page.zoomFactor = zoom;

var width = args.width || DEFAULT_WIDTH;
if (args.width) {
    console.log('Use width', args.width);
}
else {
    width = width * zoom;
}
page.viewportSize = {
    width: width,
    height: 1000
};

if (args.ua) {
    console.log('Use UA', args.ua);
}
page.settings.userAgent = args.ua || DEFAUL_UA;

var search = args.search || args.pos.join(' ');

if (!search) {
    console.log('Search not specifyed');
    phantom.exit(1);
}

var out = args.out || DEFAULT_OUT;
if (args.out) {
    console.log('Use out', args.out);
}

var delay = args.delay || 0;
if (args.delay) {
    console.log('Use delay', delay);
}

var url = 'https://www.google.com/search?q=' + encodeURIComponent(search);
console.log(url);

function render() {
    if (out == 'base64') {
        var base64 = page.renderBase64('PNG');
        system.stdout.write(base64);
    }
    else {
        page.render(out);
    }
    phantom.exit();
}

function onPageReady() {
    if (delay) {
        setTimeout(function(){
            render();
        }, delay);
    }
    else {
        render();
    }
}

page.open(url, function (status) {
    var waitInterval = window.setInterval(function () {
        var ready = page.evaluate(function() {
            var logo = document.querySelector('#logo');
            var img = document.querySelector('#logo > img');
            if (logo.style.backgroundImage) {
                logo.style.backgroundImage = logo.style.backgroundImage.replace(/(nav_logo[0-9]+)/g,'$1_hr');
                return logo.style.backgroundImage;
            }
            else if (img) {
                img.src = img.src.replace(/(nav_logo[0-9]+)/g,'$1_hr');
                return img.src;
            }
            return false;
        });
        if (ready) {
            window.clearInterval(waitInterval);
            onPageReady();
        }
    }, 100);
});
