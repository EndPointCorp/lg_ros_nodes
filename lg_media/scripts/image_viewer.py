#!/usr/bin/env python3

from threading import Lock, Thread
from functools import partial
import json
import os
import requests
import rospy
import uuid
from lg_msg_defs.msg import WindowGeometry, ApplicationState
from lg_common import ManagedApplication
from lg_common import ManagedWindow
from lg_msg_defs.msg import ImageViews, ImageView
from interactivespaces_msgs.msg import GenericMessage
from lg_common.helpers import handle_initial_state, make_soft_relaunch_callback
from lg_common.logger import get_logger
logger = get_logger('image_viewer')


def image_coordinates(image):
    return "{}_{}_{}_{}".format(image.geometry.x, image.geometry.y, image.geometry.width, image.geometry.height)


def make_key_from_image(image):
    return "{}_{}".format(image.url, image_coordinates(image))


class Image(ManagedApplication):
    def __init__(self, cmd, window, img_application, img_path, respawn=True):
        self.img_application = img_application
        self.img_path = img_path
        super(Image, self).__init__(cmd, window=window, respawn=respawn)


class ImageViewer():
    def __init__(self, viewports, save_path):
        self.current_images = {}
        self.viewports = viewports
        self.save_path = save_path
        self.lock = Lock()
        self.graphic_opts = {}
        # Per-asset (delay_seconds, duration_seconds) parsed from the scene's
        # activity_config, keyed by the ImageView uuid. Kept off the ROS message
        # (whose __slots__ are fixed) so this needs no message rebuild.
        self.asset_timing = {}
        # Incremented on every new scene. Every delay/duration timer captures the
        # generation it was scheduled under and no-ops if a newer scene has since
        # arrived -- this is how a scene change cancels pending appearances and
        # cleanups without republishing anything.
        self.scene_gen = 0

    def director_translator(self, data):
        windows_to_add = ImageViews()
        self.asset_timing = {}  # fresh per scene
        try:
            message = json.loads(data.message)
        except AttributeError:
            logger.error('Director message did not contain valid data')
            return
        except ValueError:
            logger.error('Director message did not contain valid json')
            return
        except TypeError:
            logger.error('Director message did not contai valid type. Type was %s, and content was: %s' % (type(message), message))
            return
        for window in message.get('windows', []):
            if window.get('activity', '') == 'image':
                image = ImageView()
                image.url = window['assets'][0]
                image.geometry = WindowGeometry(
                    width=window['width'],
                    height=window['height'],
                    x=window['x_coord'],
                    y=window['y_coord']
                )
                image.transparent = window.get('activity_config', {}).get('transparent', False)

                activity_config = window.get('activity_config', {})
                delay_seconds = float(activity_config.get('delay_seconds', 0) or 0)
                duration_seconds = float(activity_config.get('duration_seconds', 0) or 0)

                image.viewport = window['presentation_viewport']
                if image.viewport not in self.viewports:
                    continue
                offset_geometry = ManagedWindow.lookup_viewport_geometry(image.viewport)
                image.geometry.x = image.geometry.x + offset_geometry.x
                image.geometry.y = image.geometry.y + offset_geometry.y
                image.uuid = str(uuid.uuid4())
                self.asset_timing[image.uuid] = (delay_seconds, duration_seconds)
                if delay_seconds > 0 or duration_seconds > 0:
                    logger.info('scene asset %s: delay=%ss duration=%ss url=%s'
                                % (image.uuid, delay_seconds, duration_seconds, image.url))
                windows_to_add.images.append(image)

                self.graphic_opts[(image.url, image.geometry.x, image.geometry.y)] = {}
                self.graphic_opts[(image.url, image.geometry.x, image.geometry.y)]['no_upscale'] = window.get('activity_config', {}).get('no_upscale', False)

        logger.debug(f"Adding windows {windows_to_add}")
        self.handle_image_views(windows_to_add)

    def is_in_current_images(self, current_images, image):
        for key_from_image, _image_obj in list(current_images.items()):
            if key_from_image == make_key_from_image(image):
                return _image_obj
        return None

    def is_current_coordinates(self, current_images, image):
        for key_from_image, _image_obj in list(current_images.items()):
            logger.debug(f"comparing {'_'.join(key_from_image.split('_')[-4:])} == {image_coordinates(image)}:")
            if '_'.join(key_from_image.split('_')[-4:]) == image_coordinates(image):
                return _image_obj
        return None

    def handle_image_views(self, msg):
        with self.lock:
            self._handle_image_views(msg)

    def _timing_for(self, image):
        """(delay_seconds, duration_seconds) for an image, defaulting to (0, 0)
        for images that arrived without director timing (e.g. via /image/views)."""
        return self.asset_timing.get(image.uuid, (0.0, 0.0))

    def _remove_created_image(self, image_obj, *args, **kwargs):
        logger.debug('Removing image: {}'.format(image_obj))
        image_obj.set_state(ApplicationState.STOPPED)
        if image_obj.img_application == 'pqiv' and os.path.exists(image_obj.img_path):
            os.remove(image_obj.img_path)

    def _expire_image(self, key, gen):
        """Fired duration_seconds after an image appeared: clean it up."""
        with self.lock:
            if gen != self.scene_gen:
                logger.info('duration timer skipped for %s (scene changed)' % key)
                return  # a newer scene has superseded this one; leave it alone
            image_obj = self.current_images.pop(key, None)
            if image_obj is not None:
                logger.info('duration elapsed, removing image %s' % key)
                self._remove_created_image(image_obj)

    def _delayed_create(self, image, key, gen):
        """Fired delay_seconds after the scene arrived: bring the image up now."""
        with self.lock:
            if gen != self.scene_gen:
                logger.info('delay timer skipped for %s (scene changed)' % key)
                return  # scene changed before the asset was due; skip it entirely
            logger.info('delay elapsed, showing image %s' % key)
            created_image = self._create_image(image)
            self.current_images[key] = created_image
            _, duration_seconds = self._timing_for(image)
            if duration_seconds > 0:
                rospy.Timer(rospy.Duration(duration_seconds),
                            lambda event: self._expire_image(key, gen), oneshot=True)

    def _handle_image_views(self, msg):
        global matched_images_dict
        logger.debug("handling image views")
        self.scene_gen += 1
        gen = self.scene_gen
        new_current_images = {}
        images_to_remove = list(self.current_images.values())
        images_to_add = []
        matched_images_dict = {}
        for image in msg.images:
            delay_seconds, duration_seconds = self._timing_for(image)
            # Matching url+geometry normally keeps the image up so it does not
            # flicker between scenes. A timed image must not be kept: it would
            # skip its delay and stay on the old scene's duration clock.
            timed = delay_seconds > 0 or duration_seconds > 0
            duplicate_image = None if timed else self.is_in_current_images(self.current_images, image)
            if duplicate_image:
                logger.debug('Keeping image: {}\n\n'.format(image))
                images_to_remove.remove(duplicate_image)
                key = make_key_from_image(image)
                new_current_images[key] = duplicate_image
                continue
            current_coordinate_image = self.is_current_coordinates(self.current_images, image)
            # A delayed image must not claim the coordinate-swap: make_image (which
            # schedules the old image's teardown) does not run until the delay
            # elapses, so the old process would be orphaned. Let it tear down
            # normally instead -- a gap before a delayed asset appears is expected.
            if current_coordinate_image and delay_seconds <= 0:
                logger.debug("image matched, waiting to remove after the new one is launched")
                matched_images_dict[image_coordinates(image)] = current_coordinate_image
                images_to_remove.remove(current_coordinate_image)
            images_to_add.append(image)

        for image_obj in images_to_remove:
            self._remove_created_image(image_obj)

        def make_image(image):
            key = make_key_from_image(image)
            created_image = self._create_image(image)
            new_current_images[key] = created_image
            if image_coordinates(image) in matched_images_dict.keys():
                rospy.Timer(rospy.Duration(2), partial(self._remove_created_image, matched_images_dict[image_coordinates(image)]), oneshot=True)
            # duration counts from appearance, so schedule cleanup once it is up
            _, duration_seconds = self._timing_for(image)
            if duration_seconds > 0:
                rospy.Timer(rospy.Duration(duration_seconds),
                            lambda event, k=key: self._expire_image(k, gen), oneshot=True)

        # Create the images that appear immediately; defer the delayed ones.
        threads = []
        for image in images_to_add:
            delay_seconds, _ = self._timing_for(image)
            if delay_seconds > 0:
                continue
            thread = Thread(target=make_image, args=(image,))
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()

        self.current_images = new_current_images

        # Schedule delayed appearances after current_images is published so the
        # _delayed_create callbacks mutate the live dict.
        for image in images_to_add:
            delay_seconds, _ = self._timing_for(image)
            if delay_seconds > 0:
                key = make_key_from_image(image)
                rospy.Timer(rospy.Duration(delay_seconds),
                            lambda event, im=image, k=key: self._delayed_create(im, k, gen), oneshot=True)
        logger.debug("finished handling image views")

    def _create_image(self, image):
        if image.transparent:
            return self._create_pqiv(image)
        else:
            return self._create_feh(image)

    def _create_pqiv(self, image):
        image_path = self.save_path + '/{}'.format(image.uuid)
        r = requests.get(image.url)
        with open(image_path, 'wb') as f:
            f.write(r.content)
        opts = '-t'
        if self.graphic_opts.get((image.url, image.geometry.x, image.geometry.y), {}).get('no_upscale', False):
            opts = ''

        command = '/usr/bin/pqiv -c -i {} --scale-mode-screen-fraction=1.0 -T {} -P {},{} {}'.format(
            opts,
            image.uuid,
            image.geometry.x,
            image.geometry.y,
            image_path
        ).split()
        logger.debug('command is {}'.format(command))
        image = Image(command, ManagedWindow(w_name=image.uuid, geometry=image.geometry), img_application='pqiv', img_path=image_path)
        image.set_state(ApplicationState.STARTED)
        image.set_state(ApplicationState.VISIBLE)
        return image

    def _create_feh(self, image):
        command = '/usr/bin/feh --scale-down --image-bg black --no-screen-clip -x --title {} --geometry {}x{}+{}+{} {}'.format(
            image.uuid,
            image.geometry.width,
            image.geometry.height,
            image.geometry.x,
            image.geometry.y,
            image.url
        ).split()
        logger.debug('command is {}'.format(command))
        image = Image(command, ManagedWindow(w_name=image.uuid, geometry=image.geometry), img_application='feh', img_path=None)
        image.set_state(ApplicationState.STARTED)
        image.set_state(ApplicationState.VISIBLE)
        return image


def main():
    rospy.init_node('image_viewer')

    # logger.error('starting outputin...')
    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    save_dir = rospy.get_param('~save_dir', 'images')
    save_path = '/tmp/{}'.format(save_dir)
    if not os.path.isdir(save_path):
        os.mkdir(save_path)

    viewer = ImageViewer(viewports, save_path)

    rospy.Subscriber('/director/scene', GenericMessage, viewer.director_translator)
    rospy.Subscriber('/image/views', ImageViews, viewer.handle_image_views)

    handle_initial_state(viewer.director_translator)

    def handle_soft(*args, **kwargs):
        msg = GenericMessage()
        msg.message = '{}'
        msg.type = 'json'
        viewer.director_translator(msg)
    make_soft_relaunch_callback(handle_soft, groups=['media'])

    rospy.spin()


if __name__ == '__main__':
    main()
