var system = require('system');

var DEFAUL_UA = 'Mozilla/5.0 (X11; Linux x86_64) ' +
    'AppleWebKit/537.36 (KHTML, like Gecko) ' +
    'Chrome/53.0.2785.116 Safari/537.36';

var DEFAULT_WIDTH = 1800;
var DEFAULT_ZOOM = 1;
var DEFAULT_OUT = 'screenshot.png';

var args = require("./argparser.js").parseSysArgs();
function log() {
    if (args.silent === 'true') {
        return;
    } else {
        var argsArr = Array.prototype.slice.call(arguments);
        var strings = argsArr.map(function(o){
            return typeof o == 'string' ? o : JSON.stringify(o);
        });

        system.stderr.writeLine(strings.join(' '));
    }
}

var watchdog = setTimeout(function(){
    phantom.exit();
}, 10000);

function Screenshot() {
    this.page = require("webpage").create();
    this.args = args;

    if (typeof this.args.scripts == 'string') {
        this.args.scripts = [this.args.scripts];
    }

    if (this.args.scripts) {
        log('Applying scripts');
        for (var i = 0; i < this.args.scripts.length; i++) {
            require(this.args.scripts[i]).customize(this, log);
            log('Applyed', this.args.scripts[i]);
        }
    }

    this.initPage();
    this.initCapture();
    this.run();
}

Screenshot.prototype.initPage = function() {
    if (this.args.zoom) {
        log('Use zoom', this.args.zoom);
    }
    var zoom = this.args.zoom || DEFAULT_ZOOM;
    this.page.zoomFactor = zoom;

    var width = this.args.width || DEFAULT_WIDTH;
    if (this.args.width) {
        log('Use width', this.args.width);
    }
    else {
        width = width * zoom;
    }
    this.page.viewportSize = {
        width: width,
        height: 1000
    };

    if (this.args.ua) {
        log('Use UA', this.args.ua);
    }

    this.page.settings.userAgent = this.args.ua || DEFAUL_UA;
};

Screenshot.prototype.initCapture = function() {
    var search = this.args.search || this.args.pos.join(' ');

    if (!search && !this.args.url) {
        log('Search and url not specifyed');
        phantom.exit(1);
    }

    this.out = this.args.out || DEFAULT_OUT;
    if (this.args.out) {
        log('Use out', this.args.out);
    }

    this.delay = this.args.delay || 0;
    if (this.args.delay) {
        log('Use delay', this.delay);
    }

    this.url = this.args.url || ('https://www.google.com/search?q=' + encodeURIComponent(search));
    log(this.url);

};

Screenshot.prototype.render = function() {

    if (this.out == 'base64') {
        var base64 = this.page.renderBase64('PNG');
        system.stdout.write(base64);
    }
    else {
        this.page.render(this.out);
    }
    phantom.exit();
};

Screenshot.prototype.onPageReady = function() {
    var self = this;
    if (this.delay) {
        setTimeout(function(){
            self.render.apply(self, []);
        }, this.delay);
    }
    else {
        this.render();
    }
};

Screenshot.prototype.evaluator = function() {
    return true;
};

Screenshot.prototype.run = function() {
    var self = this;
    this.page.open(this.url, function (status) {
        var waitInterval = window.setInterval(function () {
            var ready = self.page.evaluate(self.evaluator);
            if (ready) {
                window.clearInterval(waitInterval);
                self.onPageReady.apply(self, [ready]);
            }
        }, 100);
    });
};

new Screenshot();
