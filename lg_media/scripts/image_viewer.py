#!/usr/bin/env python3

import sys
import threading
from threading import Lock, Thread
from functools import partial
import json
import os
import requests
import rospy
import uuid
import logging
import socket
import time
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

class Image(ManagedApplication):
    def __init__(self, cmd, window, img_application, img_path, respawn=True):
        self.img_application = img_application
        self.img_path = img_path
        super(Image, self).__init__(cmd, window=window, respawn=respawn)
        self.image = None  # to store ImageView object

class ImageViewer():
    def __init__(self, viewports, save_path):
        self.current_images = {}  # Keyed by geometry
        self.viewports = viewports
        self.save_path = save_path
        self.lock = Lock()
        self.graphic_opts = {}

    def director_translator(self, data):
        logger.debug("Processing director message")
        windows_to_add = ImageViews()
        try:
            message = json.loads(data.message)
        except (AttributeError, ValueError, TypeError) as e:
            logger.warning('Director message did not contain valid data: {}'.format(e))
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
                image.geometry.x += offset_geometry.x
                image.geometry.y += offset_geometry.y
                image.uuid = str(uuid.uuid4())
                windows_to_add.images.append(image)

                # Store graphic options for the image
                geometry_key = image_coordinates(image)
                self.graphic_opts[geometry_key] = {}
                activity_config = window.get('activity_config', {})
                self.graphic_opts[geometry_key]['no_upscale'] = activity_config.get('no_upscale', False)
                self.graphic_opts[geometry_key]['slideshow'] = activity_config.get('slideshow', False)
                self.graphic_opts[geometry_key]['transition'] = activity_config.get('transition', "none")
                self.graphic_opts[geometry_key]['start_delay'] = activity_config.get('start_delay', 0)
                self.graphic_opts[geometry_key]['stop_after'] = activity_config.get('stop_after', 0)

        self.handle_image_views(windows_to_add)

    def _get_socket_path_from_geometry(self, geometry):
        geometry_str = f"{geometry.width}_x_{geometry.height}_{geometry.x}_{geometry.y}".replace('-', 'm').replace('+', 'p')
        socket_path = f"/tmp/tiv_socket_{geometry_str}.sock"
        return socket_path

    def handle_image_views(self, msg):
        with self.lock:
            self._handle_image_views(msg)

    def _handle_image_views(self, msg):
        logger.debug("Handling image views")
        new_current_images = {}
        images_to_remove = self.current_images.copy()

        for image in msg.images:
            geometry_key = image_coordinates(image)
            logger.debug(f"Processing image with geometry key: {geometry_key}")

            existing_image = self.current_images.get(geometry_key, False)

            if existing_image:
                if existing_image.image.url == image.url:
                    logger.debug(f"Image with geometry {geometry_key} is already displayed; keeping it.")
                    new_current_images[geometry_key] = existing_image
                    images_to_remove.pop(geometry_key, None)
                else:
                    logger.debug(f"Updating image at geometry {geometry_key} with a new image.")
                    self._update_image(existing_image, image)
                    new_current_images[geometry_key] = existing_image
                    images_to_remove.pop(geometry_key, None)
            else:
                logger.debug(f"Creating new image at geometry {geometry_key}.")
                created_image = self._create_tiv(image)
                if created_image:
                    new_current_images[geometry_key] = created_image

        for geometry_key, image_obj in images_to_remove.items():
            logger.debug(f"Removing image at geometry {geometry_key}.")
            self._remove_image(image_obj)

        self.current_images = new_current_images
        logger.debug("Finished handling image views")

    def _remove_image(self, image_obj):
        logger.debug(f"Stopping and removing image: {image_obj}")
        image_obj.set_state(ApplicationState.STOPPED)
 
        if image_obj.img_path.startswith('/tmp/'):
            if os.path.exists(image_obj.img_path):
                try:
                    os.remove(image_obj.img_path)
                    logger.debug(f"Removed image file: {image_obj.img_path}")
                except Exception as e:
                    logger.error(f"Error removing image file {image_obj.img_path}: {e}")
 
        socket_path = self._get_socket_path_from_geometry(image_obj.image.geometry)
        if os.path.exists(socket_path):
            try:
                os.remove(socket_path)
                logger.debug(f"Removed socket file: {socket_path}")
            except Exception as e:
                logger.error(f"Error removing socket file {socket_path}: {e}")

    def _update_image(self, existing_image_obj, new_image):
        transition = self.graphic_opts.get(image_coordinates(new_image), {}).get('transition')
        if not transition:
            transition = 'fade'
        logger.debug(f"Applying transition '{transition}' to image at geometry {image_coordinates(new_image)}.")
 
        existing_image_obj.image.url = new_image.url
        self._reload_image(existing_image_obj, new_image, transition)

    def _reload_image(self, image_obj, new_image, transition):
        logger.debug(f"Reloading image in application for {image_obj}")
        image_path = self._get_local_image_path(new_image.url)
        #filename = os.path.basename(new_image.url)
        #nfs_folder = '/media/ros_cms_default_assets'
        #image_path = os.path.join(nfs_folder, filename)
        if not os.path.exists(image_path):
            # fetch it via requests and save to self.save_path
            filename = os.path.basename(image.url)
            logger.debug(f"Image not found in nfs_folder, downloading from URL: {new_image.url}")
            image_path = os.path.join(self.save_path, filename)
            try:
                r = requests.get(new_image.url)
                r.raise_for_status()
                with open(image_path, 'wb') as f:
                    f.write(r.content)
                logger.debug(f"Image downloaded and saved to: {image_path}")
            except Exception as e:
                logger.error(f"Failed to download image from {new_image.url}: {e}")
                return

        # Send command to the running image viewer process via socket
        command = f"UPDATE_IMAGE {image_path} {transition or ''}"
        socket_path = self._get_socket_path_from_geometry(image_obj.image.geometry)
        logger.debug(f"Sending command to socket {socket_path}: {command}")

        try:
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.connect(socket_path)
            client_socket.sendall(command.encode('utf-8'))
            client_socket.close()
            logger.debug("Command sent successfully")
        except Exception as e:
            logger.error(f"Failed to send command to image viewer at {socket_path}: {e}")

    def _create_tiv(self, image):
        image_path = self._get_local_image_path(image.url)
        if not os.path.exists(image_path):
            filename = os.path.basename(image.url)
            # If not, fetch it via requests and save to self.save_path
            logger.debug(f"Image not found in nfs_folder, downloading from URL: {image.url}")
            image_path = os.path.join(self.save_path, filename)
            try:
                r = requests.get(image.url)
                r.raise_for_status()
                with open(image_path, 'wb') as f:
                    f.write(r.content)
                logger.debug(f"Image downloaded and saved to: {image_path}")
            except Exception as e:
                logger.error(f"Failed to download image from {image.url}: {e}")
                return None  # Could also handle this more gracefully
        socket_path = self._get_socket_path_from_geometry(image.geometry)

        if os.path.exists(socket_path):
            logger.info(f"Socket {socket_path} already exists. Sending image to existing viewer.")
            self._send_image_to_socket(socket_path, image, transition="none")
            return None  # No need to start a new process

        command = ['/usr/bin/tiv.py']
        command.extend(['--geometry', '{}x{}+{}+{}'.format(
            image.geometry.width, image.geometry.height,
            image.geometry.x, image.geometry.y
        )])


        # Handle stretch options
        ## TODO restore stretch scale functionality/ add stretch attribute
        stretch = 'fit' if not self.graphic_opts.get(image_coordinates(image), {}).get('no_upscale', False) else 'none'
        command.extend(['--stretch', stretch])
        # Handle transition options
        transition = self.graphic_opts.get(image_coordinates(image), {}).get('transition', 'none')
        command.extend(['--transition', transition])

        start_delay = self.graphic_opts.get(image_coordinates(image), {}).get('start_delay', 0)
        if start_delay > 0:
            command.extend(['--start-delay', str(start_delay)])

        stop_after = self.graphic_opts.get(image_coordinates(image), {}).get('stop_after', 0)
        if stop_after > 0:
            command.extend(['--stop-after', str(stop_after)])
        # Handle slideshow options
        if self.graphic_opts.get(image_coordinates(image), {}).get('slideshow', False):
            # If slideshow images are specified, add them to the command
            slideshow_names = self.graphic_opts.get(image_coordinates(image)).get('slideshow', [])
            slideshow_local_paths = [self._get_local_image_path(image_name) for image_name in slideshow_names.split()] 
            if slideshow_local_paths:
                command.append('--slideshow')
                command.extend(slideshow_local_paths)
        #### here after slideshow images if any
        command.append(image_path)

        logger.info('Launching image viewer with command: {}'.format(command))
        image_obj = Image(command, ManagedWindow(w_name=image.uuid, geometry=image.geometry), img_application='tiv', img_path=image_path)
        image_obj.image = image  # Store the ImageView object for future reference
        image_obj.set_state(ApplicationState.STARTED)
        image_obj.set_state(ApplicationState.VISIBLE)
        return image_obj

    def _send_image_to_socket(self, socket_path, image, transition, retries=5, delay=0.3):
        image_path = self._get_local_image_path(image.url)
        command = f"UPDATE_IMAGE {image_path} {transition or ''}"
        logger.debug(f"Sending command to socket {socket_path}: {command}")
 
        for attempt in range(retries):
            try:
                client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                client_socket.connect(socket_path)
                client_socket.sendall(command.encode('utf-8'))
                client_socket.close()
                logger.debug("Command sent successfully")
                return
            except socket.error as e:
                logger.error(f"Failed to send command to image viewer at {socket_path}: {e}")
                if attempt < retries - 1:
                    time.sleep(delay)
                    logger.debug(f"Retrying connection to socket {socket_path} (Attempt {attempt + 2}/{retries})")
                else:
                    logger.error(f"All retries failed to connect to socket {socket_path}")

    def _get_local_image_path(self, image):
        filename = os.path.basename(image)
        nfs_folder = '/media/ros_cms_default_assets'
        image_path = os.path.join(nfs_folder, filename)
        if os.path.exists(image_path):
            return image_path
        else:
            return os.path.join(self.save_path, filename)


    @staticmethod
    def main():
        rospy.init_node('image_viewer')

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
    ImageViewer.main()

