/*
 * lg-stats js
 * Submit statistics to lg influxdb
 */

function LgStats(options) {
    var opt = options || {};
    this.host = opt.host || 'influx.galaxy.endpoint.com';
    this.port = opt.port;
    this.user = opt.user || 'jsstats';
    this.password = opt.password || 'jsstats';
    this.database = opt.database || 'jsstats';
    this.useSSL = (opt.useSSL === undefined ? true : opt.useSSL);
    this.defaultTags = opt.defaultTags || {};
    this.defaultMeasure = opt.defaultMeasure || 'jsstats';

    if (opt.url) {
        this.url = opt.url;
    }
    else {
        var protocol = this.useSSL ? 'https://' : 'http://';
        var url = protocol + this.host;
        if (this.port) {
            url += ':' + this.port;
        }
        url += '/write?db=' + this.database + '&u=' + this.user + '&p=' + this.password;
        this.url = url;
    }
}

LgStats.prototype.sendRaw = function(lineProtocolStr) {
    console.log("Send: " + lineProtocolStr);
    reqwest({
        url: this.url,
        method: 'post',
        data: lineProtocolStr,
        crossOrigin: true,
        error: function (err) {
            console.log(err);
        }
    });
};

LgStats.prototype.sendJson = function(obj) {
    var p1 = [];
    p1.push(obj.measure || this.defaultMeasure);
    var tags = encodeTags(obj);
    if (tags) {
        p1.push(tags);
    }

    var values = encodeValues(obj);

    this.sendRaw(p1.join(',') + ' ' + values);
};

LgStats.prototype.sendMsg = function(msg, measure, tags) {
    this.sendJson({
        measure: measure,
        tags: mergeOptions(this.defaultTags, tags || {}),
        values: {
            msg: msg
        }
    });
};

function encodeValues(obj) {
    var values = obj.values || {};
    var valEsc = [];
    for (var k in values) {
        if (values.hasOwnProperty(k)) {
            valEsc.push(k + '=' + escapeVal(values[k]));
        }
    }
    if (valEsc) {
        return valEsc.join(',');
    }
    return '';
}

var quoteExpr = new RegExp('"', 'g');
function escapeVal(value) {
    return '"' + value.replace(quoteExpr, '\"') + '"';
}

function encodeTags(obj) {
    var tags = obj.tags || {};
    var tagsEsc = [];
    for (var k in tags) {
        if (tags.hasOwnProperty(k)) {
            tagsEsc.push(k + '=' + escapeTagVal(tags[k]));
        }
    }
    if (tagsEsc) {
        return tagsEsc.join(',');
    }
    return '';
}

var spaceExpr = new RegExp(' ', 'g');
function escapeTagVal(val) {
    return val.replace(spaceExpr, '\ ');
}

function mergeOptions(obj1, obj2){
    var obj3 = {};
    for (var attrname in obj1) {
        obj3[attrname] = obj1[attrname];
    }
    for (var attrname in obj2) {
        obj3[attrname] = obj2[attrname];
    }
    return obj3;
}
