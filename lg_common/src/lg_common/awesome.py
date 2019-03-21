from lg_common.msg import WindowGeometry
import subprocess
import os


class WindowIdentityError(Exception):
    pass


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
    """Get a dict of rule types and values for the window.

    Args:
        window (ManagedWindow)

    Returns:
        Dict[str,str]

    Raises:
        WindowIdentityError: If the window identity can not be determined.
    """
    rules = {}
    if window.w_name is not None:
        rules['name'] = regex_escape(window.w_name)
    if window.w_instance is not None:
        rules['instance'] = regex_escape(window.w_instance)
    if window.w_class is not None:
        rules['class'] = regex_escape(window.w_class)
    if not rules:
        raise WindowIdentityError('Could not determine window identity')
    return rules


def get_rule_pattern(window):
    """Get a rule to match the given window.

    Args:
        window (ManagedWindow)

    Returns:
        str: Comma-separated awesome rules to match the window.
    """
    def pattern(r):
        return "%s = '%s'" % r
    patternized = map(pattern, get_rule_types(window).iteritems())
    return ', '.join(patternized)


def get_properties(window, chrome_kiosk_workaround=False):
    """Get a properties string to converge the window to its state.

    Args:
        window (ManagedWindow)

    Returns:
        str: Comma-separated awesome properties for the window.
    """
    prop_list = [
        "border_width = 0",
        "size_hints_honor = false",
        "floating = true",
        # Chrome Kiosk Workaround:
        # Trick Chrome in kiosk mode by setting fullscreen to true here.
        # In the callback we will set it to false.
        "fullscreen = {}".format('true' if chrome_kiosk_workaround else 'false'),
        "maximized = false",
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
    """Get an awesome callback to move the window to its proper spot.

    Args:
        window (ManagedWindow)

    Returns:
        str: awesome callback for window geometry.
    """
    if window.geometry is not None:
        return "function(c) awful.client.property.set(c, 'fullscreen', false) c:geometry({x=%d, y=%d}) end" % (
            window.geometry.x,
            window.geometry.y,
        )
    else:
        return ""


def get_entry(window, chrome_kiosk_workaround=False):
    """Get the full awesome (awful) rule that will converge the window.

    Args:
        window (ManagedWindow)

    Returns:
        str: awesome rule for the window.
    """
    rule = get_rule_pattern(window)
    properties = get_properties(window, chrome_kiosk_workaround)
    callback = get_callback(window)
    return "{ rule = { %s }, properties = { %s }, callback = %s }" % (rule, properties, callback)


def get_subtractive_script(window):
    """Get a script that will remove existing awesome (awful) rules for the
    window.

    Args:
        window (ManagedWindow)

    Returns:
        str: Lua code to remove existing rules for the window.
    """
    rules = get_rule_types(window)

    def rule(r):
        return "rule['rule']['%s'] == '%s'" % r
    checks = ' and '.join(map(rule, rules.iteritems()))
    return "for key,rule in pairs(awful.rules.rules) do if if rule['rule'] ~= nil and {checks} then table.remove(awful.rules.rules, key) end end".format(
        checks=checks
    )


def get_additive_script(window, chrome_kiosk_workaround=False):
    """Get a script that will add an awesome (awful) rule for the window.

    Args:
        window (ManagedWindow)

    Returns:
        str: Lua code to add the window rule.
    """
    entry = get_entry(window, chrome_kiosk_workaround)
    return "table.insert(awful.rules.rules, 1, {entry})".format(entry=entry)


def get_apply_script(window):
    """Get a script that will apply rules to all awesome window clients that
    match the window's identity.

    Args:
        window (ManagedWindow)

    Returns:
        str: Lua code to apply rules to all matching windows.
    """
    pattern = get_rule_pattern(window)
    return "for k,c in pairs(client.get()) do if awful.rules.match(c, {%s}) then awful.rules.apply(c) end end" % pattern


def get_script(window, chrome_kiosk_workaround=False):
    """Combine scripts to form a mega-script that fixes everything.

    Args:
        window (ManagedWindow)

    Returns:
        str: Lua code to make awesome put the window in the right place.
    """
    return ' '.join([
        "local awful = require('awful') awful.rules = require('awful.rules')",
        get_subtractive_script(window),
        get_additive_script(window, chrome_kiosk_workaround),
        get_apply_script(window)
    ])


def get_awesome_pid():
    awesome_pid = None

    try:
        awesome_pid = int(subprocess.check_output(['pidof', 'x-window-manager']))
    except:
        pass
    try:
        awesome_pid = int(subprocess.check_output(['pidof', 'awesome']))
    except:
        pass
    try:
        awesome_pid = int(subprocess.check_output(['pidof', '/usr/bin/awesome']))
    except:
        pass

    return awesome_pid


def setup_environ():
    """Attempt to copy the environment of the window manager."""
    awesome_pid = get_awesome_pid()
    if awesome_pid is None:
        raise Exception('Could not find awesome pid')

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
