import os
import xml.etree.ElementTree as ET
from xml.dom import minidom
import urllib

import rospy

CUSTOM_CONFIG_DIR = '/lg'


def get_config(base_path, instance_name, viewsync_port=42001):
    """
    Generate configuration based on ROS parameters.

    Returns:
        Tuple of configuration maps.
    """
    def curl_config(url, filename):
        """
        Curls config and writes to CUSTOM_CONFIG_DIR
        """
        response = urllib.urlopen(url)
        os.system('mkdir -p %s/%s' % (CUSTOM_CONFIG_DIR, base_path))
        with open(CUSTOM_CONFIG_DIR + '/' + base_path + '/' + filename, 'w') as f:
            f.write(response.read().replace("HOME_DIR", base_path))

    args = []

    args.extend([
        '-multiple',
        '-sConnection/disableRequestBatching=true',
        '-name', instance_name
    ])

    viewsync_send = rospy.get_param('~viewsync_send', False)
    viewsync_recv = not viewsync_send
    # default for sending should be different for receiving
    if viewsync_send:
        viewsync_port = viewsync_port
        viewsync_hostname = '127.0.0.1'
    else:
        viewsync_port = rospy.get_param('~viewsync_port_new', 42000)
        viewsync_hostname = rospy.get_param('~viewsync_host', '10.42.42.255')
    horiz_fov = rospy.get_param('~horiz_fov', 65)
    yaw_offset = rospy.get_param('~yaw_offset', 0)
    pitch_offset = rospy.get_param('~pitch_offset', 0)
    roll_offset = rospy.get_param('~roll_offset', 0)
    show_google_logo = rospy.get_param('~show_google_logo', True)
    custom_configs = rospy.get_param('~custom_configs', '')

    args.extend([
        '-sViewSync/send={}'.format(str(viewsync_send).lower()),
        '-sViewSync/receive={}'.format(str(viewsync_recv).lower()),
        '-sViewSync/hostname={}'.format(str(viewsync_hostname)),
        '-sViewSync/port={}'.format(int(viewsync_port)),
        '-sViewSync/horizFov={}'.format(float(horiz_fov)),
        '-sViewSync/yawOffset={}'.format(float(yaw_offset)),
        '-sViewSync/pitchOffset={}'.format(float(pitch_offset)),
        '-sViewSync/rollOffset={}'.format(float(roll_offset)),
    ])

    if viewsync_send:
        query_file = rospy.get_param('~query_file', '/tmp/ge_queryfile')
        args.append(
            '-sViewSync/queryFile={}'.format(query_file),
        )

    spacenav_device = rospy.get_param('~spacenav_device', None)
    spacenav_gutter = rospy.get_param('~spacenav_gutter', 0.1)
    spacenav_sensitivity = rospy.get_param('~spacenav_sensitivity', 1.0)
    spacenav_sensitivity_yaw = \
        rospy.get_param('~spacenav_sensitivity_yaw', 0.0035) * \
        spacenav_sensitivity
    spacenav_sensitivity_pitch = \
        rospy.get_param('~spacenav_sensitivity_pitch', 0.01) * \
        spacenav_sensitivity
    spacenav_sensitivity_roll = \
        rospy.get_param('~spacenav_sensitivity_roll', 0.01) * \
        spacenav_sensitivity
    spacenav_sensitivity_x = \
        rospy.get_param('~spacenav_sensitivity_x', 0.25) * \
        spacenav_sensitivity
    spacenav_sensitivity_y = \
        rospy.get_param('~spacenav_sensitivity_y', 0.25) * \
        spacenav_sensitivity
    spacenav_sensitivity_z = \
        rospy.get_param('~spacenav_sensitivity_z', 0.025) * \
        spacenav_sensitivity

    if spacenav_device is not None:
        args.extend([
            '-sSpaceNavigator/device={}'.format(
                str(spacenav_device)),
            '-sSpaceNavigator/gutterValue={}'.format(
                float(spacenav_gutter)),
            '-sSpaceNavigator/sensitivityYaw={}'.format(
                float(spacenav_sensitivity_yaw)),
            '-sSpaceNavigator/sensitivityPitch={}'.format(
                float(spacenav_sensitivity_pitch)),
            '-sSpaceNavigator/sensitivityRoll={}'.format(
                float(spacenav_sensitivity_roll)),
            '-sSpaceNavigator/sensitivityX={}'.format(
                float(spacenav_sensitivity_x)),
            '-sSpaceNavigator/sensitivityY={}'.format(
                float(spacenav_sensitivity_y)),
            '-sSpaceNavigator/sensitivityZ={}'.format(
                float(spacenav_sensitivity_z)),
            '-sSpaceNavigator/zeroYaw=0',
            '-sSpaceNavigator/zeroPitch=0',
            '-sSpaceNavigator/zeroRoll=0',
            '-sSpaceNavigator/zeroX=0',
            '-sSpaceNavigator/zeroY=0',
            '-sSpaceNavigator/zeroZ=0',
        ])

    hide_gui = rospy.get_param('~hide_gui', False)
    if hide_gui:
        args.append('--hidegui')

    geplus_config = {}

    cache_path = os.path.normpath(base_path + '/.googleearth/Cache')
    kml_path = os.path.normpath(base_path + '/.googleearth')
    flyto_speed = rospy.get_param('~flyto_speed', 0.17)
    show_compass = rospy.get_param('~show_compass', False)
    show_visualization = rospy.get_param('~show_visualization', True)

    geplus_config['General'] = {
        '3DControllerEnabled': False,
        'adsDisabled': True,
        'allowUnsafeBalloons': False,
        'AlwaysUseExternalBrowser': True,
        'buildingHighlight': False,
        'CachePath': cache_path,
        'ControllerEnabled': True,
        'ControllerMode': 2,
        'emailProvider': 0,
        'enableTips': False,
        'FlySpeed': flyto_speed,
        'GroundLevelAutoTransition': False,
        'InvertMouseWheel': False,
        'kmlErrorHandling': 0,
        'KMLPath': kml_path,
        'lastHeight': 1920,
        'lastLeft': 0,
        'lastTip': 3,
        'lastTop': 0,
        'lastWidth': 1080,
        'layersOpen': True,
        'LogoutClean': True,
        'mouseWheelSpeed': 1,
        'NavigatorShow': 0 if show_compass else 2,
        'numRuns': 1,
        'numRunsThisVersion': 1,
        'osName': 'Linux',
        'placesOpen': True,
        'PolyEditXPos': 50,
        'PolyEditYPos': '-2',
        'renderWarning-OGLsoftwareEmulated': False,
        'ReverseControls': False,
        'searchOpen': True,
        'shown_GPS': False,
        'shown_InternalBrowserWindowFrame': True,
        'shown_LeftPanel': False,
        'shown_RenderFrame': True,
        'shown_Ruler': False,
        'StoreCookies': False,
        'StreetViewNotificationShown': True,
        'SwoopEnabled': True,
        'TimeAnimSpeed': 100,
        'TimeLoopAnim': False,
        'TimeZoneHours': 0,
        'TimeZoneMinutes': 0,
        'TimeZoneMode': 1,
        'TimeZoneName': 'Local',
        'toolbarVis': False,
        'tooltips': False,
        'UsageStats': False,
        'useHttpsForGoogle2': False,
        'useHTTPSForGoogle': False,
        'UseThrownDrag': True,
        'VisualizationEnabled': show_visualization,
        'wasFullScreen': False,
        'wasMaximized': False,
    }

    use_3d_imagery = rospy.get_param('~use_3d_imagery', True)
    anisotropic_filtering = rospy.get_param('~anisotropic_filtering', 2)
    high_quality_terrain = rospy.get_param('~high_quality_terrain', True)
    texture_compression = rospy.get_param('~texture_compression', True)
    status_bar_visible = rospy.get_param('~status_bar_visible', True)

    geplus_config['Render'] = {
        '3DImageryEnabled': use_3d_imagery,
        'AnisotropicFiltering_6_2': anisotropic_filtering,
        'AnisotropicFiltering': anisotropic_filtering,
        'Antialiasing': 0,
        'Atmosphere': True,
        'DisableAdvancedFeatures': False,
        'ElevationExaggeration': 1,
        'FeetMiles': False,
        'GridReference': 1,
        'HighQualityTerrain': high_quality_terrain,
        'IconSize': 1,
        'MeasurementUnits': 2,
        'OverviewSize': 34,
        'OverviewZoom': 99,
        'PrimaryFontVersion3Family': 'Arial',
        'PrimaryFontVersion3Size': 14,
        'PrimaryFontVersion3Style': 0,
        'PrimaryFontVersion3Weight': 75,
        'RenderingApi': 1,
        'StatusBarVisible': status_bar_visible,
        'TerrainEnabled': True,
        'TerrainQuality': 0,
        'TextureColors': 1,
        'TextureCompressionDXTC': texture_compression,
        'TextureCompression': texture_compression,
        'WaterSurface': True,
    }

    mem_cache_size = rospy.get_param('~mem_cache_size', 64)
    disk_cache_size = rospy.get_param('~disk_cache_size', 256)

    geplus_config['Cache'] = {
        'MemoryCacheSize': mem_cache_size,
        'DiskCacheSize': disk_cache_size,
    }

    geplus_config['Layer'] = {
        'drivingDirectionsRange': 100,
        'drivingDirectionsSpeed': 150,
        'drivingDirectionsTilt': 60,
        'tourBalloonShow': False,
        'tourFlysAlongsLines': False,
        'tourFlyTime': 10,
        'tourRecordingAccuracy': 75,
        'tourTrackKeyframeSpacing': 3,
        'tourTrackSpeed': 6,
        'tourWaitTime': 3,
    }

    geplus_config['TourGuide'] = {
        'Filmstrip\Enabled': False,
        'Filmstrip\Expanded': False,
    }

    layers_config = {}

    show_state_borders = rospy.get_param('~show_state_borders', True)
    show_country_borders = rospy.get_param('~show_country_borders', True)
    show_state_labels = rospy.get_param('~show_state_labels', True)
    show_country_labels = rospy.get_param('~show_country_labels', True)
    show_city_labels = rospy.get_param('~show_city_labels', True)
    show_water_labels = rospy.get_param('~show_water_labels', False)
    show_gray_buildings = rospy.get_param('~show_gray_buildings', False)
    show_buildings = rospy.get_param('~show_buildings', True)
    show_trees = rospy.get_param('~show_trees', True)

    layers_config['LayersVisibility'] = {
        # Earth: Alphabetical order
        r'http%3A__kh.google.com%3A80_\1st%20Level%20Admin%20Borders%20%28States_Provinces%29': show_state_borders,
        r'http%3A__kh.google.com%3A80_\1st%20Level%20Admin%20Names%20%28States_Provinces%29': show_state_labels,
        r'http%3A__kh.google.com%3A80_\2002%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2003%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2004%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2005%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2006%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2007%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2008%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2008%20-%20Cloud%20Cover%20%2811-50%25%29': False,
        r'http%3A__kh.google.com%3A80_\2008%20-%20Cloud%20Cover%20%2851%2B%25%29': False,
        r'http%3A__kh.google.com%3A80_\2009%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2009%20-%20Cloud%20Cover%20%2811-50%25%29': False,
        r'http%3A__kh.google.com%3A80_\2009%20-%20Cloud%20Cover%20%2851%2B%25%29': False,
        r'http%3A__kh.google.com%3A80_\2010%20-%20Cloud%20Cover%20%280-10%25%29': False,
        r'http%3A__kh.google.com%3A80_\2010%20-%20Cloud%20Cover%20%2811-50%25%29': False,
        r'http%3A__kh.google.com%3A80_\2010%20-%20Cloud%20Cover%20%2851%2B%25%29': False,
        r'http%3A__kh.google.com%3A80_\2nd%20Level%20Admin%20Regions%20%28Counties%29': False,
        r'http%3A__kh.google.com%3A80_\360%20Cities': False,
        r'http%3A__kh.google.com%3A80_\Airports': False,
        r'http%3A__kh.google.com%3A80_\Banks_ATMs': False,
        r'http%3A__kh.google.com%3A80_\Bars_Clubs': False,
        r'http%3A__kh.google.com%3A80_\Bus': False,
        r'http%3A__kh.google.com%3A80_\Campsites': False,
        r'http%3A__kh.google.com%3A80_\Cemeteries': False,
        r'http%3A__kh.google.com%3A80_\Churches': False,
        r'http%3A__kh.google.com%3A80_\City%20Boundaries': False,
        r'http%3A__kh.google.com%3A80_\Coastlines': False,
        r'http%3A__kh.google.com%3A80_\Coffee%20Shops': False,
        r'http%3A__kh.google.com%3A80_\Country%20Names': show_country_labels,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Asian': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Barbecue': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Fast%20Food': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Indian': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Italian': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Japanese': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Mexican': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Other': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Pizza': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Seafood': False,
        r'http%3A__kh.google.com%3A80_\Dining%20-%20Steak%20Houses': False,
        r'http%3A__kh.google.com%3A80_\Elementary%20School%20Districts': False,
        r'http%3A__kh.google.com%3A80_\Fire': False,
        r'http%3A__kh.google.com%3A80_\Forest%20Service%20Facilities': False,
        r'http%3A__kh.google.com%3A80_\Gas%20Stations': False,
        r'http%3A__kh.google.com%3A80_\GeoEye%20Featured%20Imagery': False,
        r'http%3A__kh.google.com%3A80_\Golf%20Courses': False,
        r'http%3A__kh.google.com%3A80_\Gray%20': show_gray_buildings,
        r'http%3A__kh.google.com%3A80_\Grocery%20Stores': False,
        r'http%3A__kh.google.com%3A80_\Hospitals': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\2010%20FIFA%20World%20Cup%U2122': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\360Cities': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\About%20the%20Maps': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Activities': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Adventure': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Africa%20Megaflyover': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Ancient%20Rome%203D': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Astronaut%20Photography%20of%20Earth': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Atlas%20Tour%3A%20China%2C%20Italy%2C%20Brazil%2C%20Australia': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Attractions': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Destinations': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Disney%20Resorts': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Disney%27s%20Animal%20Kingdom%AE%20Park': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Disney%27s%20Hollywood%20Studios%U2122': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Disney%27s%20Wide%20World%20of%20Sports%AE': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Disneyland%AE%20Resort%20Paris': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Earth%20beauty%20seen%20from%20space': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Earth%20City%20Lights': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Earthquakes': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Entertainment': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Epcot%AE': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Everytrail': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Feature%20Articles%20%26%20Photographs': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Food%20and%20Drinks': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Gigapan%20Photos': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Gigapxl%20Photos': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Google%20Earth%20Community': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Heritage%20%26%20Culture': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Heritage': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Highlights': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Information': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Japan%20Tourism': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Korea%20Tourism': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Kyoto%20Tourism': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Live%20WildCams': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Lodging': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Magic%20Kingdom%AE': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Map%20Finder': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Multimedia': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Nature': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\New%20York%20Times': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Nile%20Cruises': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Phenomena%20seen%20from%20space': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Places': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Road%20Markers%20and%20Conditions': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Routes': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Safari': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Satellite%20Imagery': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Scenic%20Highlights': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Seaside': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Shopping%20and%20Services': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Sights%20%26%20Sounds': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Ski%20resorts%20in%20the%20Alps': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Sunrise%20Earth': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Trimble%20Outdoors%20Trips': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Urban%20Vibes': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Visitor%20Centers': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Volcanoes': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Webcams.travel': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\Wikiloc': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\YouTube': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_gallery_layers_gallery_root_en.kmz\ZipUSA': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_geographic_features_en.kml\Geographic%20Features': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Appalachian%20Mountaintop%20Removal': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\ARKive%3A%20Endangered%20Species': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Earthwatch%20Expeditions': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Fair%20Trade%20Certified': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Global%20Heritage%20Fund': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Greenpeace': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Jane%20Goodall%27s%20Gombe%20Chimpanzee%20Blog': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\The%20Earth%20from%20Above%20with%20GoodPlanet': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\The%20Elders%3A%20Every%20Human%20Has%20Rights': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\UNDP%3A%20Millennium%20Development%20Goals%20Monitor': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\UNEP%3A%20Atlas%20of%20Our%20Changing%20Environment': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\Unicef%3A%20Water%20and%20Sanitation': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\USHMM%3A%20Crisis%20in%20Darfur': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\USHMM%3A%20World%20is%20Witness': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\WaterAid': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_outreach_root2_root_en.kmz\WWF%20Conservation%20Projects': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\3D%20cities': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\Earth%20View%20landscapes': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\https%3A__mw1.google.com_mw-earth-vectordb_voyager_root_teaser-en.kmz': False,
        r'http%3A__kh.google.com%3A80_\https%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\Finding%20Home': False,
        r'http%3A__kh.google.com%3A80_\https%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\Happy%2010th%20Birthday%2C%20Google%20Earth%21': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\Satellite%20imagery%20updates': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\Street%20View%20highlights': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\%3Ca%20href_%22https%3A__mw1.google.com_mw-earth-vectordb_voyager_root_nl-voyager-en.kmz%22%3EDownload%3C_a%3E': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-earth-vectordb_voyager_root_voyager-en.kmz\%3Ca%20href_%22%23voyager_tour%3Bflyto%22%3EHighlight%20tour%3C_a%3E': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-weather_base_files_kml_weather_en.kmz\Clouds': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-weather_base_files_kml_weather_en.kmz\Conditions%20and%20Forecasts': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-weather_base_files_kml_weather_en.kmz\Information': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-weather_base_files_kml_weather_en.kmz\Radar': False,
        r'http%3A__kh.google.com%3A80_\International%20Borders': show_country_borders,
        r'http%3A__kh.google.com%3A80_\Islands': False,
        r'http%3A__kh.google.com%3A80_\Local%20Place%20Names': False,
        r'http%3A__kh.google.com%3A80_\Lodging': False,
        r'http%3A__kh.google.com%3A80_\Lookouts': False,
        r'http%3A__kh.google.com%3A80_\Major%20Retail': False,
        r'http%3A__kh.google.com%3A80_\Mosques': False,
        r'http%3A__kh.google.com%3A80_\Mountain%20Rail': False,
        r'http%3A__kh.google.com%3A80_\Movie_DVD%20Rental': False,
        r'http%3A__kh.google.com%3A80_\National%20Forest%20Boundaries': False,
        r'http%3A__kh.google.com%3A80_\Other%20Places%20of%20Worship': False,
        r'http%3A__kh.google.com%3A80_\Panoramio': False,
        r'http%3A__kh.google.com%3A80_\Park%20Boundaries': False,
        r'http%3A__kh.google.com%3A80_\Park%20Descriptions': False,
        r'http%3A__kh.google.com%3A80_\Parks%20': False,
        r'http%3A__kh.google.com%3A80_\Pharmacy': False,
        r'http%3A__kh.google.com%3A80_\Photorealistic%20': show_buildings,
        r'http%3A__kh.google.com%3A80_\Picnic%20Areas': False,
        r'http%3A__kh.google.com%3A80_\Places%20': False,
        r'http%3A__kh.google.com%3A80_\Populated%20Places': show_city_labels,
        r'http%3A__kh.google.com%3A80_\Postal%20Code%20Boundaries': False,
        r'http%3A__kh.google.com%3A80_\Rail': False,
        r'http%3A__kh.google.com%3A80_\Roadside%20Park': False,
        r'http%3A__kh.google.com%3A80_\Roads': False,
        r'http%3A__kh.google.com%3A80_\Schools': False,
        r'http%3A__kh.google.com%3A80_\Secondary%20School%20Districts': False,
        r'http%3A__kh.google.com%3A80_\Shopping%20Malls': False,
        r'http%3A__kh.google.com%3A80_\SPOT%20One%20World%2C%20One%20Year': False,
        r'http%3A__kh.google.com%3A80_\Subway': False,
        r'http%3A__kh.google.com%3A80_\Synagogues': False,
        r'http%3A__kh.google.com%3A80_\Temples': False,
        r'http%3A__kh.google.com%3A80_\Traffic': False,
        r'http%3A__kh.google.com%3A80_\Trail%20Junctions': False,
        r'http%3A__kh.google.com%3A80_\Trailhead': False,
        r'http%3A__kh.google.com%3A80_\Trails': False,
        r'http%3A__kh.google.com%3A80_\Tram': False,
        r'http%3A__kh.google.com%3A80_\Trees': show_trees,
        r'http%3A__kh.google.com%3A80_\Unified%20School%20Districts': False,
        r'http%3A__kh.google.com%3A80_\US%20Congressional%20Districts': False,
        r'http%3A__kh.google.com%3A80_\US%20Fish%20and%20Wildlife%20Service%20boundary': False,
        r'http%3A__kh.google.com%3A80_\US%20Senators': False,
        r'http%3A__kh.google.com%3A80_\Visitor%20Facilities': False,
        r'http%3A__kh.google.com%3A80_\Water%20Bodies': show_water_labels,
        r'http%3A__kh.google.com%3A80_\Water%20Body%20Outlines': False,
        r'http%3A__kh.google.com%3A80_\Waterway': False,
        r'http%3A__kh.google.com%3A80_\Wikipedia': False,
        r'http%3A__kh.google.com%3A80_\Winter%20Recreation%20Area': False,
        # Ocean: Alphabetical order
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Animal%20Tracking': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Arctic%20Sea%20Ice': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\ARKive%3A%20Endangered%20Ocean%20Species': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Census%20of%20Marine%20Life': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Cousteau%20Ocean%20World': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Dead%20Zones': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Dive%20Spots': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Explore%20the%20Ocean': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Human%20Impacts': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Kite%20Surfing%20Spots': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Magazine%20Quiz': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Marie%20Tharp%20Historical%20Map': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Marine%20Protected%20Areas': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\MBA%3A%20Seafood%20Watch': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\MCS%3A%20Fish%20to%20Eat': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Ocean%20Atlas': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Ocean%20Expeditions': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Ocean%20Observations': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Sea%20Surface%20Temperature': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Shipwrecks': False,
        r'http%3A__kh.google.com%3A80_\http%3A__mw1.google.com_mw-ocean_ocean_root_1_root_en.kmz\Surf%20Spots': False,
        # Moon: Alphabetical order
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_featured_images.kml\Featured%20Satellite%20Images': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_gallery.kml\Apollo%20Missions': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_gallery.kml\Guided%20Tours': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_gallery.kml\Historic%20Maps': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_gallery.kml\Human%20Artifacts': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_global_maps.kml\Colorized%20Terrain': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_global_maps.kml\Lunar%20Orbiter%20Mosaic': False,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_global_maps.kml\Visible%20Imagery': True,
        r'http%3A__khmdb.google.com%3A80__db_moon\http%3A__mw1.google.com_mw-planetary_moon1_placenames.kml\Place%20Names': False,
        # Mars: Alphabetical order
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_featured_images.kml\Featured%20Satellite%20Images': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_gallery.kml\A%20Traveler%27s%20Guide%20to%20Mars': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_gallery.kml\Guided%20Tours': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_gallery.kml\Historic%20Maps': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_gallery.kml\Live%20from%20Mars': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_gallery.kml\Rovers%20and%20Landers': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\Colorized%20Terrain': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\Daytime%20Infrared': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\MDIM%202.1%20Image%20Map': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\Nighttime%20Infrared': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\Viking%20Color%20Imagery': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_global_maps.kml\Visible%20Imagery': True,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_imagery_root.kml\CRISM%20Image%20Browser': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_imagery_root.kml\CTX%20Image%20Browser': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_imagery_root.kml\HiRISE%20Image%20Browser': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_imagery_root.kml\HRSC%20Image%20Browser': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_imagery_root.kml\MOC%20Image%20Browser': False,
        r'http%3A__khmdb.google.com%3A80__db_mars\http%3A__mw1.google.com_mw-earth-vectordb_planetary_mars1_placenames.kml\Place%20Names': False,
    }

    kml_sync_base = rospy.get_param('~kml_sync_base', None)
    kml_sync_slug = rospy.get_param('~kml_sync_slug', 'default')

    kml_root = ET.Element('kml', attrib={})
    kml_root.attrib['xmlns'] = 'http://www.opengis.net/kml/2.2'
    kml_root.attrib['xmlns:gx'] = 'http://www.google.com/kml/ext/2.2'
    kml_document = ET.SubElement(kml_root, 'Document')
    kml_folder = ET.SubElement(kml_document, 'Folder')
    kml_folder_name = ET.SubElement(kml_folder, 'name').text = 'My Places'
    kml_folder_name = ET.SubElement(kml_folder, 'open').text = '1'

    if kml_sync_base is not None:
        kml_sync_folder = ET.SubElement(kml_folder, 'Folder')

        kml_sync_master_nl = ET.SubElement(kml_sync_folder, 'NetworkLink')
        ET.SubElement(kml_sync_master_nl, 'name').text = 'Master KML CMS'
        kml_sync_master_nl_link = ET.SubElement(kml_sync_master_nl, 'Link')
        ET.SubElement(kml_sync_master_nl_link, 'href').text = \
            '{}/master.kml'.format(kml_sync_base)

        kml_sync_update_nl = ET.SubElement(kml_sync_folder, 'NetworkLink')
        ET.SubElement(kml_sync_update_nl, 'name').text = 'KML Update CMS'
        kml_sync_update_nl_link = ET.SubElement(kml_sync_update_nl, 'Link')
        ET.SubElement(kml_sync_update_nl_link, 'href').text = \
            '{}/network_link_update.kml?window_slug={}'.format(
                kml_sync_base.rstrip('/'), kml_sync_slug)
        ET.SubElement(kml_sync_update_nl_link, 'refreshMode').text = \
            'onInterval'
        ET.SubElement(kml_sync_update_nl_link, 'refreshInterval').text = \
            '1'
        ET.SubElement(kml_sync_update_nl_link, 'viewRefreshMode').text = \
            'onStop'
        ET.SubElement(kml_sync_update_nl_link, 'viewRefreshTime').text = \
            '1'
        ET.SubElement(kml_sync_update_nl_link, 'viewFormat').text = \
            'bboxWest=[bboxWest]&bboxSouth=[bboxSouth]&' + \
            'bboxEast=[bboxEast]&bboxNorth=[bboxNorth]&' + \
            'lookatLon=[lookatLon]&lookatLat=[lookatLat]&' + \
            'lookatRange=[lookatRange]&lookatTilt=[lookatTilt]&' + \
            'lookatHeading=[lookatHeading]&cameraLon=[cameraLon]&' + \
            'cameraLat=[cameraLat]&cameraAlt=[cameraAlt]'

    kml_reparsed = minidom.parseString(ET.tostring(kml_root))
    kml_content = kml_reparsed.toprettyxml(indent='\t')

    default_view = rospy.get_param(
        '~default_view',

        '<LookAt><longitude>-122.4661297737901</longitude>' +
        '<latitude>37.71903477888115</latitude><altitude>0</altitude>' +
        '<heading>42.60360249388481</heading>' +
        '<tilt>66.02791701475958</tilt><range>36611.51655091633</range>' +
        '<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode></LookAt>'
    )

    # the parser requires namespaces to be declared
    default_view_fix = \
        '<kml xmlns:gx="http://www.google.com/kml/ext/2.2">{}</kml>'.format(default_view)
    default_view_parsed = minidom.parseString(default_view_fix)
    default_view_lat = \
        default_view_parsed.getElementsByTagName('latitude')[0].childNodes[0].data
    default_view_lng = \
        default_view_parsed.getElementsByTagName('longitude')[0].childNodes[0].data

    view_root = ET.Element('kml', attrib={
        'xmlns': 'http://www.opengis.net/kml/2.2',
        'xmlns:gx': 'http://www.google.com/kml/ext/2.2',
    })
    view_document = ET.SubElement(view_root, 'Document')
    ET.SubElement(view_document, 'name').text = 'cached_default_view.kml'

    view_placemark = ET.SubElement(view_document, 'Placemark', attrib={
        'id': 'default_starting_location'
    })
    ET.SubElement(view_placemark, 'name').text = 'Starting Location'

    view_point = ET.SubElement(view_placemark, 'Point')
    ET.SubElement(view_point, 'coordinates').text = '{},{}'.format(
        default_view_lat, default_view_lng)

    ET.SubElement(view_placemark, 'dummy')

    view_reparsed = minidom.parseString(ET.tostring(view_root).replace(
        '<dummy />', default_view))
    view_content = view_reparsed.toprettyxml(indent='\t')

    for url_and_filename in custom_configs.split(';'):
        if not url_and_filename:
            continue
        split = url_and_filename.split(',')
        if len(split) < 2:
            continue
        url = split[0]
        filename = split[1]
        curl_config(url, filename)

    return args, geplus_config, layers_config, kml_content, view_content


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
