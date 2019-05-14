#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from threading import Lock
import json
import os
import requests
import rospy
import uuid
from lg_common.msg import WindowGeometry, ApplicationState
from lg_common import ManagedApplication
from lg_common import ManagedWindow
from lg_media.msg import ImageViews, ImageView
from interactivespaces_msgs.msg import GenericMessage


class Image(ManagedApplication):
    def __init__(self, cmd, window, respawn=True):
        super(Image, self).__init__(cmd, window=window, respawn=respawn)


class ImageViewer():
    def __init__(self, viewports, save_path):
        self.current_images = {}
        self.viewports = viewports
        self.save_path = save_path
        self.lock = Lock()

    def director_translator(self, data):
        windows_to_add = ImageViews()
        try:
            message = json.loads(data.message)
        except AttributeError:
            rospy.logwarn('Director message did not contain valid data')
            return
        except ValueError:
            rospy.logwarn('Director message did not contain valid json')
            return
        except TypeError:
            rospy.logwarn('Director message did not contai valid type. Type was %s, and content was: %s' % (type(message), message))
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
                image.viewport = window['presentation_viewport']
                if image.viewport not in self.viewports:
                    continue
                offset_geometry = ManagedWindow.lookup_viewport_geometry(image.viewport)
                image.geometry.x = image.geometry.x + offset_geometry.x
                image.geometry.y = image.geometry.y + offset_geometry.y
                image.uuid = str(uuid.uuid4())
                windows_to_add.images.append(image)
        self.handle_image_views(windows_to_add)

    def is_in_current_images(self, current_images, image):
        for _image, _image_obj in current_images.items():
            if _image.url == image.url and \
                    _image.geometry == image.geometry:
                return _image_obj
        return None

    def handle_image_views(self, msg):
        with self.lock:
            self._handle_image_views(msg)

    def _handle_image_views(self, msg):
        new_current_images = {}
        images_to_remove = self.current_images.values()
        images_to_add = []
        for image in msg.images:
            # rospy.logerr('CURRENT IMAGES: {}\n\n'.format(self.current_images))
            duplicate_image = self.is_in_current_images(self.current_images, image)
            if duplicate_image:
                # rospy.logerr('Keeping image: {}\n\n'.format(image))
                images_to_remove.remove(duplicate_image)
                new_current_images[image] = duplicate_image
                continue
            rospy.logwarn('Appending IMAGE: {}\n\n'.format(image))
            images_to_add.append(image)

        for image_obj in images_to_remove:
            image_obj.set_state(ApplicationState.STOPPED)
            file_to_remove = image_obj.cmd[-1]
            if os.path.exists(file_to_remove):
                os.remove(file_to_remove)
        #images_to_remove = []
        for image in images_to_add:
            new_current_images[image] = self._create_image(image)

        self.current_images = new_current_images

    def _create_image(self, image):
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
        #command = '/usr/bin/feh --bg trans -x --title {} --geometry {}x{}+{}+{} {}'.format(
        #    u,
        #    image.geometry.width,
        #    image.geometry.height,
        #    image.geometry.x,
        #    image.geometry.y,
        #    image.url
        #).split()
        rospy.logwarn('command is {}'.format(command))
        image = Image(command, ManagedWindow(w_name=image.uuid, geometry=image.geometry))
        image.set_state(ApplicationState.STARTED)
        image.set_state(ApplicationState.VISIBLE)
        return image


def main():
    rospy.init_node('image_viewer')

    # rospy.logerr('starting outputin...')
    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    save_dir = rospy.get_param('~save_dir', 'images')
    save_path = '/tmp/{}'.format(save_dir)
    if not os.path.isdir(save_path):
        os.mkdir(save_path)

    viewer = ImageViewer(viewports, save_path)

    rospy.Subscriber('/director/scene', GenericMessage, viewer.director_translator)
    rospy.Subscriber('/image/views', ImageViews, viewer.handle_image_views)

    rospy.spin()


if __name__ == '__main__':
    main()
