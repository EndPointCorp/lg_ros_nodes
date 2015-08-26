from lg_common import ManagedWindow
from lg_media.msg import AdhocMedia
from lg_media.msg import AdhocMedias
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import extract_first_asset_from_director_message

class DirectorMediaBridge():
    """
    Bridge between director and MplayerPool or BrowserPlayerPool on specified viewport_name

    Depending on activity name, messages will contain `media_type` of `video` (mplayer) or `browser_player` (browser e.g. popcorn.js)

    Listens on director messages and emits AdhocMedias messages via `mplayer_pool_publisher`
    """

    def __init__(self, adhoc_media_pool_publisher, viewport_name):
        """
        MediaDirectorBridge should be configured per each viewport to properly translate director geometry to viewport geometry and
        provide separation and service granularity
        """
        self.viewport_name = viewport_name
        self.adhoc_media_pool_publisher

    def translate_director(self, data):
        """
        Translates director messages to AdhocMedias message
        """
        adhoc_media_list = self._extract_adhoc_media(data)

        adhoc_media_instances = AdhocMedias()
        adhoc_medias.medias = adhoc_media_list
        rospy.logdebug("Publishing AdhocMedias: %s" % adhoc_medias)

    def _extract_adhoc_media(self, data):
        """
        Returns a list containing AdhocMedia objects extracted from director message for specified viewport
        specific to adhoc_media that this instance of bridge is configured for
        """
        adhoc_medias = []

        mplayer_medias = extract_first_asset_from_director_message(data, 'video', self.viewport_name)
        browser_medias = extract_first_asset_from_director_message(data, 'browser_video', self.viewport_name)

        mplayer_adhoc_medias = self._build_adhoc_medias(mplayer_medias, 'video')
        browser_adhoc_medias = self._build_adhoc_medias(mplayer_medias, 'browser_video')

        adhoc_medias.append(mplayer_adhoc_medias)
        adhoc_medias.append(browser_adhoc_medias)

        self.adhoc_media_pool_publisher.publish(adhoc_medias)

    def _build_adhoc_medias(self, media_list, media_type):
        """
        Accepts json medias list and converts them into AdhocMedias per any media type
        """
        adhoc_medias = []
        media_id = 0
        for adhoc_media in media_list:
            adhoc_media_name = 'adhoc_media_' + media_type + '_' + self.viewport_name + '_' + str(media_id)
            adhoc_media = AdhocMedia()
            adhoc_media.id = adhoc_media_name
            adhoc_media.url = adhoc_media['path']
            adhoc_media.geometry.x = adhoc_media['x_coord'] + self._get_viewport_offset()['x']
            adhoc_media.geometry.y = adhoc_media['y_coord'] + self._get_viewport_offset()['x']
            adhoc_media.width = adhoc_media['width']
            adhoc_media.height = adhoc_media['height']
            adhoc_media.media_type = media_type

            adhoc_medias.append(adhoc_media)
            media_id += 1

        rospy.logdebug("Returning adhocmedias: %s for player: %s" % (adhoc_medias, media_type))
        return adhoc_medias

    def _get_viewport_offset(self):
        """
        Director messages contain offset and size local to the screen that media should be displayed on.
        Screens have their own offsets that should be honored during ManagedWindow creation.
        This method will return `real` offset
        """
        viewport_geometry = ManagedWindow.get_viewport_geometry()
        if not viewport_geometry:
            viewport_geometry = WindowGeometry()

        return {'x': viewport_geometry.x, 'y': viewport_geometry.y}
