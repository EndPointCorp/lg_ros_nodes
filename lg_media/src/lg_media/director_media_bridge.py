import collections
import rospy
import threading

from lg_common import ManagedWindow
from lg_msg_defs.msg import AdhocMedia
from lg_msg_defs.msg import AdhocMedias
from lg_msg_defs.msg import WindowGeometry
from lg_common.helpers import extract_first_asset_from_director_message
from .asset_timing import mark_timed_id
from lg_common.logger import get_logger
logger = get_logger('director_media_bridge')


class DirectorMediaBridge():
    """
    Bridge between director and MplayerPool or BrowserPlayerPool on specified viewport_name
    Depending on activity name, messages will contain
        `media_type` of `video` (mplayer) or `browser_player` (browser e.g. popcorn.js)
    Listens on director messages and emits AdhocMedias messages via `mplayer_pool_publisher`

    """

    def __init__(self, adhoc_media_pool_publisher, viewport_name, media_type='video', media_pool=None):
        """
        MediaDirectorBridge should be configured per each viewport to properly translate
        director geometry to viewport geometry and provide separation and service granularity.

        media_pool is the pool this bridge feeds, when it lives in the same
        node (mplayer, gstreamer). Per-asset `delay_seconds`/`duration_seconds`
        are then driven by adding/removing single assets on it directly.

        browser_video has no local pool -- its consumer is a separate node -- so
        there the bridge instead publishes the set of assets that should be up
        right now, and the downstream pool diffs it.
        """
        self.viewport_name = viewport_name
        self.adhoc_media_pool_publisher = adhoc_media_pool_publisher
        self.media_type = media_type
        self.media_pool = media_pool
        self.lock = threading.Lock()
        # Per-asset (delay_seconds, duration_seconds), keyed by asset id.
        self.asset_timing = {}
        # Assets that should be up right now, keyed by id. Only used when there
        # is no local pool to drive directly.
        self.due = {}
        # Incremented on every scene. Every delay/duration timer captures the
        # generation it was scheduled under and no-ops if a newer scene has
        # since arrived -- that is how a scene change cancels pending
        # appearances and cleanups without republishing anything.
        self.scene_gen = 0

    def _timing_for(self, media_id):
        """(delay_seconds, duration_seconds) for an asset, defaulting to (0, 0)."""
        return self.asset_timing.get(media_id, (0.0, 0.0))

    def _delayed_create(self, media, gen):
        """Fired delay_seconds after the scene arrived: start the asset now."""
        with self.lock:
            if gen != self.scene_gen:
                logger.info('delay timer skipped for media %s (scene changed)' % media.id)
                return  # scene changed before the asset was due; skip it entirely
            logger.info('delay elapsed, showing media %s (%s)' % (media.id, media.url))
            if self.media_pool is not None:
                self.media_pool.add_timed_media(media)
            else:
                self.due[media.id] = media
                self._publish_due()
            _, duration_seconds = self._timing_for(media.id)
            if duration_seconds > 0:
                self._schedule_expiry(media.id, duration_seconds, gen)

    def _expire_media(self, media_id, gen):
        """Fired duration_seconds after an asset appeared: tear it down."""
        with self.lock:
            if gen != self.scene_gen:
                logger.info('duration timer skipped for media %s (scene changed)' % media_id)
                return  # a newer scene has superseded this one; leave it alone
            if self.media_pool is not None:
                if self.media_pool.remove_timed_media(media_id):
                    logger.info('duration elapsed, removing media %s' % media_id)
            elif self.due.pop(media_id, None) is not None:
                logger.info('duration elapsed, removing media %s' % media_id)
                self._publish_due()

    def _publish_due(self):
        """Publish exactly the assets that should be up right now.

        Used when there is no local pool: the downstream node turns these into
        browsers and its pool diffs them, so dropping an asset from this list
        tears it down and adding one brings it up.
        """
        self.adhoc_media_pool_publisher.publish(AdhocMedias(medias=list(self.due.values())))

    def _schedule_expiry(self, media_id, duration_seconds, gen):
        rospy.Timer(rospy.Duration(duration_seconds),
                    lambda event: self._expire_media(media_id, gen),
                    oneshot=True)

    def translate_director(self, data):
        """
        Translates director messages to AdhocMedias message.

        """
        with self.lock:
            self.scene_gen += 1
            gen = self.scene_gen
            adhoc_medias = self._extract_adhoc_media(data)

            # Assets with a delay are held back from this publish entirely --
            # the pool must not see them yet, or it would start them now. They
            # are added to the pool one by one as their timers fire.
            immediate = [m for m in adhoc_medias.medias if self._timing_for(m.id)[0] <= 0]
            deferred = [m for m in adhoc_medias.medias if self._timing_for(m.id)[0] > 0]

            logger.debug("Publishing AdhocMedias: %s" % immediate)
            self.due = collections.OrderedDict((m.id, m) for m in immediate)
            self.adhoc_media_pool_publisher.publish(AdhocMedias(medias=immediate))

            # Duration counts from when the asset appears, and the immediate
            # ones are up as of this publish.
            for media in immediate:
                _, duration_seconds = self._timing_for(media.id)
                if duration_seconds > 0:
                    self._schedule_expiry(media.id, duration_seconds, gen)

            for media in deferred:
                delay_seconds, _ = self._timing_for(media.id)
                rospy.Timer(rospy.Duration(delay_seconds),
                            lambda event, m=media: self._delayed_create(m, gen),
                            oneshot=True)

    def _extract_adhoc_media(self, data):
        """
        Returns a list containing AdhocMedia objects extracted from director
        message for specified viewport specific to adhoc_media that this
        instance of bridge is configured for.

        """
        # first get assets
        medias = extract_first_asset_from_director_message(data, self.media_type, self.viewport_name)
        logger.debug("Got assets for %s based media player %s" % (self.media_type, medias))
        # and wrap them inside AdhocMedia
        adhoc_medias = self._build_adhoc_medias(medias, self.media_type)
        logger.debug("I'm going to publish following adhoc_medias: %s" % adhoc_medias)
        # finally return list of AdhocMedia in AdhocMedias message
        return AdhocMedias(medias=adhoc_medias)

    def _build_adhoc_medias(self, media_list, media_type):
        """
        Accepts json medias list and converts them into AdhocMedias per any media type.

        """
        adhoc_medias = []
        media_id = 0
        self.asset_timing = {}  # fresh per scene
        for media in media_list:
            activity_config = media.get('activity_config', {})
            delay_seconds = float(activity_config.get('delay_seconds', 0) or 0)
            duration_seconds = float(activity_config.get('duration_seconds', 0) or 0)

            media_name = 'adhoc_media_' + media_type + '_' + self.viewport_name + '_' + str(media_id)
            if delay_seconds > 0 or duration_seconds > 0:
                # The scene counter makes the id differ every scene, so a timed
                # asset is torn down and restarted rather than carried over --
                # consumers keyed on a stable id would otherwise keep it alive.
                # mark_timed_id stays last: is_timed_id() tests the suffix.
                media_name = mark_timed_id('%s_s%d' % (media_name, self.scene_gen))

            adhoc_media = AdhocMedia()
            adhoc_media.id = media_name
            adhoc_media.url = media['path']
            adhoc_media.geometry.x = media['x_coord'] + self._get_viewport_offset()['x']
            adhoc_media.geometry.y = media['y_coord'] + self._get_viewport_offset()['y']
            adhoc_media.geometry.width = media['width']
            adhoc_media.geometry.height = media['height']
            adhoc_media.media_type = media_type
            # TODO figure out if media['on_finish'] or media['activity_config']['onFinish'] is better
            adhoc_media.on_finish = media['on_finish']
            adhoc_media.extra_args = activity_config.get('args', '')
            adhoc_medias.append(adhoc_media)

            self.asset_timing[media_name] = (delay_seconds, duration_seconds)
            if delay_seconds > 0 or duration_seconds > 0:
                logger.info('scene media %s: delay=%ss duration=%ss url=%s'
                            % (media_name, delay_seconds, duration_seconds, adhoc_media.url))

            media_id += 1

        logger.debug("Returning adhocmedias: %s for player: %s" % (adhoc_medias, media_type))
        return adhoc_medias

    def _get_viewport_offset(self):
        """
        Director messages contain offset and size local to the screen that media
        should be displayed on.
        Screens have their own offsets that should be honored during ManagedWindow creation.
        This method will return `real` offset.

        """
        viewport_geometry = ManagedWindow.get_viewport_geometry()
        if not viewport_geometry:
            viewport_geometry = WindowGeometry()
        return {'x': viewport_geometry.x, 'y': viewport_geometry.y}
