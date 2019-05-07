#!/usr/bin/env python
import json
import rospy
import uuid
from lg_media.msg import ImageViews, ImageView
from interactivespaces_msgs.msg import GenericMessage
from lg_common.msg import WindowGeometry, ApplicationState
from lg_common import ManagedApplication
from lg_common import ManagedWindow


class Image(ManagedApplication):
    def __init__(self, cmd, window, respawn=True):
        super(Image, self).__init__(cmd, window=window, respawn=respawn)

class ImageViewer():
    def __init__(self, image_view_pub, viewports, master=False):
        self.current_images = {}
        self.image_view_pub = image_view_pub
        self.viewports = viewports
        self.master = master

    def director_translator(self, data):
        if not self.master:
            return
        windows_to_add = ImageViews()
        try:
            message = json.loads(data.message)
        except AttributeError:
            rospy.logwarn("Director message did not contain valid data")
            return
        except ValueError:
            rospy.logwarn("Director message did not contain valid json")
            return
        except TypeError:
            rospy.logwarn("Director message did not contai valid type. Type was %s, and content was: %s" % (type(message), message))
            return
        rospy.logerr("we are here...")
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
                offset_geometry = ManagedWindow.lookup_viewport_geometry(image.viewport)
                image.geometry.x = image.geometry.x + offset_geometry.x
                image.geometry.y = image.geometry.y + offset_geometry.y
                windows_to_add.images.append(image)
        self.image_view_pub.publish(windows_to_add)

    #def _compare_images(self, created_image, window):
    #    return created_image.geometry.width == window['width'] and \
    #           created_image.geometry.height == window['height'] and \
    #           created_image.geometry.x == window['x'] and \
    #           created_image.geometry.y == window['y'] and \
    #           created_image.path == window['assets'][0]

    def handle_image_views(self, msg):
        # current_images =  {ImageView.msg: Image() }
        # print "we are here..."
        new_current_images = {}
        images_to_remove = self.current_images.values()
        images_to_add = []
        for image in msg.images:
            rospy.logdebug("CURRENT IMAGES: {}\n\n".format(self.current_images))
            if image.viewport not in self.viewports:
                continue
            if image in self.current_images.keys() and image in images_to_remove:
                images_to_remove.remove(self.current_images[image])
                new_current_images[image] = self.current_images[image]
                continue
            images_to_add.append(image)

        for image in images_to_add:
            new_current_images[image] = self._create_image(image)
        for image_obj in images_to_remove:
            image_obj.set_state(ApplicationState.STOPPED)

        self.current_images = new_current_images
        print "current images {}".format(self.current_images)





        #for window in self.windows_to_add:
        #    for image in self.current_images:
        #        if _compare_images(image, window):
        #            self.current_images.remove(image)
        #            self.images_to_remove.remove(image)
        #        else:
        #            self.images_to_add.append(_create_image(window))
        #views = ImageViews(self.images_to_add)


    def _create_image(self, image):
        # TODO compile uuid and set window name
        u = str(uuid.uuid4())
        command = '/usr/bin/feh -x --title {} --geometry {}x{}+{}+{} {}'.format(
            u,
            image.geometry.width,
            image.geometry.height,
            image.geometry.x,
            image.geometry.y,
            image.url
        ).split()
        rospy.logerr( "command is {}".format(command))
        image = Image(command, ManagedWindow(w_name=u, geometry=image.geometry))
        image.set_state(ApplicationState.STARTED)
        image.set_state(ApplicationState.VISIBLE)
        return image


def main():
    rospy.init_node('image_viewer')

    rospy.logerr("starting outpin...")
    image_view_pub = rospy.Publisher('/image/views', ImageViews, queue_size=10)
    viewports = [param.strip() for param in rospy.get_param('~viewports', '').split(',')]
    master = rospy.get_param('~master', False)

    viewer = ImageViewer(image_view_pub, viewports, master)

    rospy.Subscriber('/director/scene', GenericMessage, viewer.director_translator)
    rospy.Subscriber('/image/views', ImageViews, viewer.handle_image_views)
    print "about to spin..."

    rospy.spin()


if __name__ == '__main__':
    main()
