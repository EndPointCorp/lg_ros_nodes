import copy
import xml.etree.ElementTree as ET


KML_NS = 'http://www.opengis.net/kml/2.2'
GX_NS = 'http://www.google.com/kml/ext/2.2'

ET.register_namespace('', KML_NS)
ET.register_namespace('gx', GX_NS)


def _tag(namespace, name):
    return '{{{}}}{}'.format(namespace, name)


def _local_name(tag):
    return tag.rsplit('}', 1)[-1]


def pose_camera_kml(pose):
    return (
        '<Camera><longitude>{}</longitude><latitude>{}</latitude>'
        '<altitude>{}</altitude><heading>{}</heading><tilt>{}</tilt>'
        '<roll>{}</roll><altitudeMode>absolute</altitudeMode></Camera>'
    ).format(
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.z, pose.orientation.x, pose.orientation.y,
    )


def pose_lookat_kml(pose):
    return (
        '<LookAt><longitude>{}</longitude><latitude>{}</latitude>'
        '<altitude>{}</altitude><heading>{}</heading><tilt>{}</tilt>'
        '<range>{}</range>'
        '<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>'
    ).format(
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.z, pose.orientation.x, pose.orientation.y,
    )


def parse_flyto(fragment, duration=0):
    """Normalize a Camera, LookAt, or gx:FlyTo fragment."""
    fragment = fragment.strip()
    if fragment.startswith('flytoview='):
        fragment = fragment[len('flytoview='):].lstrip()

    wrapped = '<root xmlns="{}" xmlns:gx="{}">{}</root>'.format(
        KML_NS, GX_NS, fragment)
    try:
        wrapper = ET.fromstring(wrapped)
    except ET.ParseError as exc:
        raise ValueError('invalid KML fragment: {}'.format(exc))

    if len(wrapper) != 1:
        raise ValueError('expected exactly one FlyTo, Camera, or LookAt element')

    element = wrapper[0]
    name = _local_name(element.tag)
    if name in ('Camera', 'LookAt'):
        flyto = ET.Element(_tag(GX_NS, 'FlyTo'))
        ET.SubElement(flyto, _tag(GX_NS, 'duration')).text = str(duration)
        ET.SubElement(flyto, _tag(GX_NS, 'flyToMode')).text = 'bounce'
        flyto.append(element)
        return flyto

    if name != 'FlyTo':
        raise ValueError('expected FlyTo, Camera, or LookAt; got {}'.format(name))

    element.tag = _tag(GX_NS, 'FlyTo')
    duration_element = None
    for child in element:
        child_name = _local_name(child.tag)
        if child_name in ('duration', 'flyToMode'):
            child.tag = _tag(GX_NS, child_name)
        if child_name == 'duration':
            duration_element = child
    if duration_element is None:
        duration_element = ET.Element(_tag(GX_NS, 'duration'))
        element.insert(0, duration_element)
    duration_element.text = str(duration)

    if not any(_local_name(child.tag) in ('Camera', 'LookAt')
               for child in element):
        raise ValueError('FlyTo must contain a Camera or LookAt')
    return element


def build_tour_kml(fragment, tour_name, duration=0):
    """Build a self-playing KML tour around one camera movement."""
    flyto = parse_flyto(fragment, duration=duration)
    root = ET.Element(_tag(KML_NS, 'kml'), {'id': tour_name})
    document = ET.SubElement(root, _tag(KML_NS, 'Document'))

    # Define the tour before its autoplay link. Earth resolves NetworkLinks
    # while parsing, so the opposite order can race playtour against tour
    # registration.
    tour = ET.SubElement(document, _tag(GX_NS, 'Tour'))
    ET.SubElement(tour, _tag(KML_NS, 'name')).text = tour_name
    playlist = ET.SubElement(tour, _tag(GX_NS, 'Playlist'))
    playlist.append(flyto)

    network_link = ET.SubElement(document, _tag(KML_NS, 'NetworkLink'))
    link = ET.SubElement(network_link, _tag(KML_NS, 'Link'))
    ET.SubElement(link, _tag(KML_NS, 'href')).text = (
        'http://localhost:8765/query.html?query=playtour={}'.format(tour_name))
    ET.SubElement(link, _tag(KML_NS, 'refreshMode')).text = 'onChange'
    ET.SubElement(link, _tag(KML_NS, 'viewRefreshMode')).text = 'never'

    return '<?xml version="1.0" encoding="UTF-8"?>\n{}'.format(
        ET.tostring(root, encoding='unicode'))


DEFAULT_CENTER_EARTH = {
    'activity': 'earth',
    'activity_config': {},
    'assets': [],
    'height': 1920,
    'presentation_viewport': 'center',
    'slug': -1875729098,
    'width': 1080,
    'x_coord': 0,
    'y_coord': 0,
}


def attach_center_tour(scene, url, url_prefix, viewport='center'):
    """Return a scene with one managed tour asset on center Earth only."""
    scene = copy.deepcopy(scene)
    center = None
    for window in scene.setdefault('windows', []):
        if (window.get('activity') in ('earth', 'lg_earth') and
                window.get('presentation_viewport') == viewport):
            center = window
            break
    if center is None:
        center = copy.deepcopy(DEFAULT_CENTER_EARTH)
        center['presentation_viewport'] = viewport
        scene['windows'].append(center)

    center['assets'] = [asset for asset in center.get('assets', [])
                        if not asset.startswith(url_prefix)]
    center['assets'].append(url)

    # The base router must preserve Cesium while this hidden Earth asset is
    # refreshed for background following.
    scene['preserve_base'] = True
    return scene
