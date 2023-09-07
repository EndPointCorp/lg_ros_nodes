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

    def director_translator(self, data):
        logger.error("ZZZ")
        windows_to_add = ImageViews()
        try:
            message = json.loads(data.message)
        except AttributeError:
            logger.warning('Director message did not contain valid data')
            return
        except ValueError:
            logger.warning('Director message did not contain valid json')
            return
        except TypeError:
            logger.warning('Director message did not contai valid type. Type was %s, and content was: %s' % (type(message), message))
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
                image.viewport = window['presentation_viewport']
                if image.viewport not in self.viewports:
                    continue
                offset_geometry = ManagedWindow.lookup_viewport_geometry(image.viewport)
                image.geometry.x = image.geometry.x + offset_geometry.x
                image.geometry.y = image.geometry.y + offset_geometry.y
                image.uuid = str(uuid.uuid4())
                windows_to_add.images.append(image)
        logger.error(f"ZZZ {windows_to_add}")
        self.handle_image_views(windows_to_add)

    def is_in_current_images(self, current_images, image):
        for key_from_image, _image_obj in list(current_images.items()):
            if key_from_image == make_key_from_image(image):
                return _image_obj
        return None

    def is_current_coordinates(self, current_images, image):
        for key_from_image, _image_obj in list(current_images.items()):
            logger.error(f"ZZZ comparing {'_'.join(key_from_image.split('_')[-4:])} == {image_coordinates(image)}:")
            if '_'.join(key_from_image.split('_')[-4:]) == image_coordinates(image):
                return _image_obj
        return None

    def handle_image_views(self, msg):
        with self.lock:
            self._handle_image_views(msg)

    def _handle_image_views(self, msg):
        logger.error("handling image views")
        new_current_images = {}
        images_to_remove = list(self.current_images.values())
        images_to_add = []
        images_to_remove_delayed = []
        for image in msg.images:
            # logger.error('CURRENT IMAGES: {}\n\n'.format(self.current_images))
            duplicate_image = self.is_in_current_images(self.current_images, image)
            if duplicate_image:
                # logger.error('Keeping image: {}\n\n'.format(image))
                images_to_remove.remove(duplicate_image)
                new_current_images[make_key_from_image(image)] = duplicate_image
                continue
            current_coordinate_image = self.is_current_coordinates(self.current_images, image)
            if current_coordinate_image:
                logger.error("image matched, sending to background")
                images_to_remove_delayed.append(current_coordinate_image)
                images_to_remove.remove(current_coordinate_image)
            logger.debug('Appending IMAGE: {}\n\n'.format(image))
            images_to_add.append(image)

        def remove_image(image_obj, *args, **kwargs):
            logger.error('Removing image: {}'.format(image_obj))
            image_obj.set_state(ApplicationState.STOPPED)
            if image_obj.img_application == 'pqiv' and os.path.exists(image_obj.img_path):
                os.remove(image_obj.img_path)

        for image_obj in images_to_remove:
            logger.error('ZZZ removing image object')
            remove_image(image_obj)
        #images_to_remove = []
        threads = []

        def make_image(image):
            created_image = self._create_image(image)
            new_current_images[make_key_from_image(image)] = created_image

        for image in images_to_add:
            thread = Thread(target=make_image, args=(image,))
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()
        
        for image_obj in images_to_remove_delayed:
            logger.error('ZZZ delaying removal of imagei 0.1 seconds')
            rospy.Timer(rospy.Duration(.1), partial(remove_image, image_obj), oneshot=True)

        self.current_images = new_current_images
        logger.error("finished handling image views")

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
        command = '/usr/bin/pqiv -c -i -T {} -P {},{} {}'.format(
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
        command = '/usr/bin/feh --image-bg black --no-screen-clip -x --title {} --geometry {}x{}+{}+{} {}'.format(
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
