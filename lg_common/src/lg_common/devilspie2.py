import subprocess
import tempfile


def _regex_escape(raw):
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


def _get_condition(w_name, w_class, w_instance):
    conditions = []
    if w_name:
        conditions.append('string.match(get_window_name(), [[{}]])'.format(_regex_escape(w_name)))
    if w_class:
        conditions.append('string.match(get_application_name(), [[{}]])'.format(_regex_escape(w_class)))
    if w_instance:
        conditions.append('string.match(get_class_instance_name(), [[{}]])'.format(_regex_escape(w_instance)))

    return '\n\tand '.join(conditions)


def _get_properties(is_visible):
    properties = [
        'set_window_fullscreen(false)',
        'undecorate_window()',
        'unmaximize()',
        'unminimize()' if is_visible else 'minimize()',
    ]
    return '\n'.join(properties)


def _get_geometry_setter(geometry):
    lines = []
    if geometry:
        lines.append('set_window_geometry2({x}, {y}, {w}, {h})'.format(x=geometry.x, y=geometry.y, w=geometry.width, h=geometry.height))
    return '\n'.join(lines)


class Devilspie2:
    def __init__(self, w_name, w_class, w_instance, geometry, is_visible):
        condition = _get_condition(w_name, w_class, w_instance)
        properties = _get_properties(is_visible)
        geometry_setter = _get_geometry_setter(geometry)

        script = '''
if {condition} then
debug_print("found the window\\n")
{properties}
{geometry_setter}
end
        '''.format(
            condition=condition,
            properties=properties,
            geometry_setter=geometry_setter,
        ).strip()

        self._config_dir = tempfile.TemporaryDirectory(prefix='ManagedWindow')
        with open(self._config_dir.name + '/devilespie2.lua', 'w') as f:
            f.write(script)

        self._proc = subprocess.Popen(
            args=['devilspie2', '-f', self._config_dir.name],
            stdin=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def close(self):
        self._config_dir.cleanup()
        if self._proc:
            self._proc.terminate()
            self._proc = None

    def __del__(self):
        self.close()
