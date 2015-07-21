from lg_common.msg import WindowGeometry
import subprocess
import os

def regex_escape(raw):
    """Escape lua regex."""
    escaped = raw
    escaped = escaped.replace("%", "%%")
    escaped = escaped.replace("^", "%^")
    escaped = escaped.replace("$", "%$")
    escaped = escaped.replace("(", "%(")
    escaped = escaped.replace(")", "%)")
    escaped = escaped.replace("[", "%[")
    escaped = escaped.replace("]", "%]")
    escaped = escaped.replace(".", "%.")
    escaped = escaped.replace("*", "%*")
    escaped = escaped.replace("+", "%+")
    escaped = escaped.replace("-", "%-")
    escaped = escaped.replace("?", "%?")
    escaped = escaped.replace("\0", "%z")
    return escaped

def get_rule_types(window):
    """Get a list of tuples with rule types and values."""
    rules = []
    if window.w_name is not None:
        rules.append(('name', regex_escape(window.w_name)))
    if window.w_instance is not None:
        rules.append(('instance', regex_escape(window.w_instance)))
    if window.w_class is not None:
        rules.append(('class', regex_escape(window.w_class)))
    if len(rules) == 0:
        raise Exception('Could not determine window identity')
    return rules

def get_rule_pattern(window):
    def pattern(rule):
        return "%s = '%s'" % rule
    patternized = map(pattern, get_rule_types(window))
    return ', '.join(patternized)

def get_properties(window):
    prop_list = [
        "border_width = 0",
        "size_hints_honor = false",
        "floating = true",
        "hidden = {}".format('false' if window.is_visible else 'true'),
        "minimized = {}".format('false' if window.is_visible else 'true'),
        "opacity = {}".format(1 if window.is_visible else 0),
    ]
    if window.geometry is not None:
        prop_list.extend([
            "width = {}".format(window.geometry.width),
            "height = {}".format(window.geometry.height),
        ])
    return ', '.join(prop_list)

def get_callback(window):
    if window.geometry is not None:
        return "function(c) c:geometry({x=%d, y=%d}) end" % (
            window.geometry.x,
            window.geometry.y,
        )
    else:
        return ""

def get_entry(window):
    rule = get_rule_pattern(window)
    properties = get_properties(window)
    callback = get_callback(window)
    return "{ rule = { %s }, properties = { %s }, callback = %s }" % (rule, properties, callback)

def get_subtractive_script(window):
    rules = get_rule_types(window)
    def match(rule):
        return "rule['rule']['%s'] == '%s'" % rule
    checks = ' and '.join(map(match, rules))
    return "for key,rule in pairs(awful.rules.rules) do if {checks} then table.remove(awful.rules.rules, key) end end".format(
        checks=checks
    )

def get_additive_script(window):
    entry = get_entry(window)
    return "table.insert(awful.rules.rules, 1, {entry})".format(entry=entry)

def get_apply_script(window):
    pattern = get_rule_pattern(window)
    return "for k,c in pairs(client.get()) do if awful.rules.match(c, {%s}) then awful.rules.apply(c) end end" % pattern

def get_script(window):
    return ' '.join([
        "local awful = require('awful') awful.rules = require('awful.rules')",
        get_subtractive_script(window),
        get_additive_script(window),
        get_apply_script(window)
    ])

def setup_environ():
    awesome_pid = int(subprocess.check_output(['pidof', 'x-window-manager']))
    with open('/proc/{}/environ'.format(awesome_pid), 'r') as f:
        awesome_environ_raw = f.read().strip('\0')
    def split_environ(raw):
        pair = raw.split('=', 1)
        return pair[0], pair[1]
    pairs = map(split_environ, awesome_environ_raw.split('\0'))
    awesome_environ = dict((p[0], p[1]) for p in pairs)
    # TODO(mv): return environment for Popen instead of messing with parent environment
    def copy_environ(k):
        os.environ[k] = awesome_environ[k]
    copy_environ('DISPLAY')
    copy_environ('DBUS_SESSION_BUS_ADDRESS')
    copy_environ('XAUTHORITY')

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
