module.exports = {
    parseSysArgs: parseSysArgs
};

function parseSysArgs() {

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

    return args;
}
