#!/usr/bin/env python3

import sys
import argparse
import os
import time
import logging
import threading
import socket
from PyQt5.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QPoint, QEasingCurve, pyqtSignal

nfs_path = '/media/ros_cms_default_assets'
# Set up logging
log_directory = "/home/lg/log/"  # Replace with your log directory
log_file = os.path.join(log_directory, "tiv_log.txt")

if not os.path.exists(log_directory):
    os.makedirs(log_directory)

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file),
        # Uncomment the next line to enable console logging
        # logging.StreamHandler()
    ]
)

logger = logging.getLogger('tiv')

def parse_arguments():
    parser = argparse.ArgumentParser(description='Custom Image Viewer')
    parser.add_argument('images', nargs='*', help='Image file(s) to display')
    parser.add_argument('--geometry', help='Window geometry as WIDTHxHEIGHT+X+Y', default=None)
    parser.add_argument('--stretch', choices=['scale', 'fit', 'none'], help='Stretch image to scale or fit window', default="none")
    parser.add_argument('--slideshow', action='store_true', help='Play images as a slideshow')
    parser.add_argument('--slideshow-interval', type=int, default=5, help='Slideshow interval in seconds')
    parser.add_argument('--start-delay', type=int, default=0, help='Start delay in seconds')
    parser.add_argument('--stop-after', type=int, default=0, help='Stop after specified seconds')
    parser.add_argument('--transition', choices=['none', 'fade', 'slide_left', 'slide_right', 'slide_up', 'slide_down'], help='Transition effect between images', default="none")
    parser.add_argument('--transparent', action='store_true', default='true', help='Make window background transparent')
    return parser.parse_args()

class ImageViewer(QWidget):
    update_image_signal = pyqtSignal(str, str)  # image_path, transition

    def __init__(self, args):
        super().__init__()
        logger.info("Initializing ImageViewer...")
        self.args = args
        self.images = args.images or []
        self.current_image_index = 0
        self.current_transition = args.transition
        self.slideshow_timer = None
        self.ipc_thread = None
        self.animation = None

        self.initUI()
        self.socket_path = self.get_socket_path()

        if args.start_delay > 0:
            logger.debug(f"Starting viewer after delay of {args.start_delay} seconds")
            self.hide()
            QTimer.singleShot(args.start_delay * 1000, self.show_viewer)
        else:
            self.show()

        if args.stop_after > 0:
            logger.debug(f"Viewer will stop after {args.stop_after} seconds")
            QTimer.singleShot(args.stop_after * 1000, self.close)

        if args.slideshow and len(self.images) > 1:
            logger.debug(f"Starting slideshow with interval {args.slideshow_interval} seconds")
            self.slideshow_timer = QTimer()
            self.slideshow_timer.timeout.connect(self.next_image)
            self.slideshow_timer.start(self.args.slideshow_interval * 1000)

        # Start IPC server thread
        self.ipc_thread = threading.Thread(target=self.ipc_server)
        self.ipc_thread.daemon = True
        self.ipc_thread.start()

        # Connect signals
        self.update_image_signal.connect(self.perform_update_from_signal)

    def show_viewer(self):
        logger.debug("Showing viewer after delay")
        self.show()

    def initUI(self):
        logger.info("Initializing UI...")
        self.label_current = QLabel(self)
        self.label_current.setAlignment(Qt.AlignCenter)
        self.label_current.setStyleSheet("background-color: rgba(0, 0, 0, 0);")
        self.label_current.setGeometry(0, 0, self.width(), self.height())

        self.label_next = QLabel(self)
        self.label_next.setAlignment(Qt.AlignCenter)
        self.label_next.setStyleSheet("background-color: rgba(0, 0, 0, 0);")
        self.label_next.setGeometry(0, 0, self.width(), self.height())
        self.label_next.hide()

        if self.args.transparent:
            self.setAttribute(Qt.WA_TranslucentBackground, True)
            self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.X11BypassWindowManagerHint)
        else:
            self.setWindowFlags(Qt.FramelessWindowHint)

        if self.args.geometry:
            geom = self.parse_geometry(self.args.geometry)
            logger.debug(f"Setting geometry: {geom}")
            self.setGeometry(*geom)

        if self.images:
            self.load_image(self.images[self.current_image_index])

    def parse_geometry(self, geometry):
        import re
        pattern = r'(\d+)x(\d+)\+(\-?\d+)\+(\-?\d+)'
        match = re.match(pattern, geometry)
        if match:
            width, height, x, y = map(int, match.groups())
            return x, y, width, height
        else:
            logger.warning(f"Invalid geometry format: {geometry}")
            # Default geometry
            return 100, 100, 800, 600

    def get_socket_path(self):
        geometry_str = f"{self.width()}_x_{self.height()}_{self.x()}_{self.y()}".replace('-', 'm').replace('+', 'p')
        socket_path = f"/tmp/tiv_socket_{geometry_str}.sock"
        return socket_path

    def load_image(self, image_path):
        logger.debug(f"Loading image: {image_path}")
        if not os.path.exists(image_path):
            logger.error(f"Image {image_path} does not exist.")
            return

        pixmap = QPixmap()
        if not pixmap.load(image_path):
            logger.error(f"Failed to load image: {image_path}")
            return
        pixmap.detach()  # Ensure the image is reloaded from disk

        if self.args.stretch == 'scale':
            logger.debug("Stretching image to scale")
            pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        elif self.args.stretch == 'fit':
            logger.debug("Stretching image to fit")
            pixmap = pixmap.scaled(self.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        else:
            logger.debug("Not stretching the image")

        self.label_current.setPixmap(pixmap)
        self.label_current.setGeometry(0, 0, self.width(), self.height())
        self.label_current.show()
        self.label_current.repaint()
        self.repaint()
        self.update()

    def next_image(self):
        logger.debug("Advancing to next image")
        if len(self.images) > 1:
            self.current_image_index = (self.current_image_index + 1) % len(self.images)
            next_image_path = self.images[self.current_image_index]
            self.update_current_image(next_image_path, self.current_transition)

    def perform_transition(self, image_path):
        logger.debug(f"perform_transition called with image_path: {image_path} and current_transition: {self.current_transition}")
        if self.current_transition == 'fade':
            self.fade_transition(image_path)
        elif self.current_transition and self.current_transition.startswith('slide'):
            direction = self.current_transition.split('_')[1]
            self.slide_transition(image_path, direction)
        else:
            # No transition, directly load the image
            logger.debug("No transition specified, directly loading image")
            self.load_image(image_path)

    def fade_transition(self, image_path):
        logger.debug("Performing fade transition")
        pixmap = self.prepare_pixmap(image_path)
        if pixmap is None:
            return

        self.label_next.setPixmap(pixmap)
        self.label_next.setGeometry(0, 0, self.width(), self.height())
        self.label_next.setGraphicsEffect(QGraphicsOpacityEffect(self.label_next))
        self.label_next.graphicsEffect().setOpacity(0)
        self.label_next.show()

        self.animation = QPropertyAnimation(self.label_next.graphicsEffect(), b"opacity")
        self.animation.setDuration(500)
        self.animation.setStartValue(0)
        self.animation.setEndValue(1)
        self.animation.finished.connect(self.on_fade_finished)
        self.animation.start()

    def on_fade_finished(self):
        self.label_current.setPixmap(self.label_next.pixmap())
        self.label_current.repaint()
        self.repaint()
        self.update()
        self.label_next.hide()
        self.label_next.setPixmap(QPixmap())
        self.label_next.setGraphicsEffect(None)
        self.animation = None

    def slide_transition(self, image_path, direction):
        logger.debug(f"Performing slide transition from {direction}")
        pixmap = self.prepare_pixmap(image_path)
        if pixmap is None:
            return

        self.label_next.setPixmap(pixmap)
        self.label_next.setGeometry(0, 0, self.width(), self.height())
        self.label_next.show()

        width = self.width()
        height = self.height()

        if direction == 'left':
            start_pos = QPoint(width, 0)
            end_pos = QPoint(0, 0)
        elif direction == 'right':
            start_pos = QPoint(-width, 0)
            end_pos = QPoint(0, 0)
        elif direction == 'up':
            start_pos = QPoint(0, height)
            end_pos = QPoint(0, 0)
        elif direction == 'down':
            start_pos = QPoint(0, -height)
            end_pos = QPoint(0, 0)
        else:
            logger.error(f"Unknown slide direction: {direction}")
            return

        self.label_next.move(start_pos)
        self.label_next.raise_()

        self.animation = QPropertyAnimation(self.label_next, b"pos")
        self.animation.setDuration(500)
        self.animation.setStartValue(start_pos)
        self.animation.setEndValue(end_pos)
        self.animation.setEasingCurve(QEasingCurve.InOutQuad)
        self.animation.finished.connect(self.on_slide_finished)
        self.animation.start()

    def on_slide_finished(self):
        self.label_current.setPixmap(self.label_next.pixmap())
        self.label_current.repaint()
        self.repaint()
        self.update()
        self.label_next.hide()
        self.label_next.setPixmap(QPixmap())
        self.animation = None

    def prepare_pixmap(self, image_path):
        if not os.path.exists(image_path):
            logger.error(f"Image {image_path} does not exist.")
            return None

        pixmap = QPixmap()
        if not pixmap.load(image_path):
            logger.error(f"Failed to load image: {image_path}")
            return None
        pixmap.detach()  # Ensure the image is reloaded from disk

        if self.args.stretch == 'scale':
            pixmap = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        elif self.args.stretch == 'fit':
            pixmap = pixmap.scaled(self.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        return pixmap

    def update_image(self, image_path, transition):
        self.current_transition = transition or "none"
        logger.debug(f"Updating image to {image_path} with transition {transition}")
        self.images = [image_path]
        self.current_image_index = 0
        self.update_image_signal.emit(image_path, transition or "none")
        if self.slideshow_timer:
            self.slideshow_timer.stop()
            self.slideshow_timer = None

    def update_current_image(self, image_path, transition):
        self.current_transition = transition
        logger.debug(f"Updating current image to {image_path} with transition {transition}")
        self.update_image_signal.emit(image_path, transition)

    def ipc_server(self):
        if os.path.exists(self.socket_path):
            try:
                os.remove(self.socket_path)
            except:
                pass
        server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server_socket.bind(self.socket_path)
        server_socket.listen(5)
        logger.debug(f"IPC server listening on {self.socket_path}")

        while True:
            try:
                conn, _ = server_socket.accept()
                data = conn.recv(4096).decode('utf-8').strip()
                logger.debug(f"Received IPC command: {data}")
                if data.startswith("UPDATE_IMAGE"):
                    parts = data.split(' ', 2)
                    if len(parts) >= 2:
                        image_path = parts[1]
                        transition = parts[2] if len(parts) > 2 else "none"
                        self.update_image(image_path, transition)
                elif data.startswith("UPDATE_SLIDESHOW"):
                    parts = data.split(' ', 1)
                    if len(parts) >= 2:
                        image_paths = parts[1].split()
                        self.update_slideshow(image_paths)
                conn.close()
            except Exception as e:
                logger.error(f"Error in IPC server: {e}", exc_info=True)
                continue

        server_socket.close()
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)
            logger.debug(f"Socket file {self.socket_path} removed")

    def update_slideshow(self, image_paths):
        logger.debug(f"Updating slideshow with images: {image_paths}")
        self.images = image_paths
        self.current_image_index = 0
        if self.slideshow_timer:
            self.slideshow_timer.stop()
        else:
            self.slideshow_timer = QTimer()
            self.slideshow_timer.timeout.connect(self.next_image)
        self.slideshow_timer.start(self.args.slideshow_interval * 1000)
        self.update_current_image(self.images[self.current_image_index], self.current_transition)

    def closeEvent(self, event):
        if os.path.exists(self.socket_path):
            try:
                os.remove(self.socket_path)
                logger.debug(f"Socket file {self.socket_path} removed")
            except:
                pass
        super().closeEvent(event)

    def resizeEvent(self, event):
        logger.debug("Window resized")
        self.label_current.setGeometry(0, 0, self.width(), self.height())
        self.label_next.setGeometry(0, 0, self.width(), self.height())
        super().resizeEvent(event)

    def perform_update_from_signal(self, image_path, transition):
        self.current_transition = transition
        self.perform_transition(image_path)

def main():
    args = parse_arguments()
    app = QApplication(sys.argv)
    try:
        viewer = ImageViewer(args)
        sys.exit(app.exec_())
    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)

if __name__ == '__main__':
    main()

