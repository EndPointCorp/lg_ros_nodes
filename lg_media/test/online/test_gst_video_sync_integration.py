#!/usr/bin/env python

PKG = 'lg_media'
NAME = 'test_gst_video_sync_integration'

import unittest
import rospy
import json
import subprocess
import tempfile
import wave
from interactivespaces_msgs.msg import GenericMessage
from contextlib import contextmanager


@contextmanager
def generate_silence():
    with tempfile.NamedTemporaryFile() as f:
        wav = wave.open(f.name, mode='wb')
        wav.setsampwidth(2)
        wav.setnchannels(1)
        wav.setframerate(8000)
        wav.writeframes(bytes([0 * 800000]))
        wav.close()
        yield f.name


class TestGstVideoSyncIntegration(unittest.TestCase):
    def setUp(self):
        self.scene_pub = rospy.Publisher('/director/scene', GenericMessage, queue_size=10)
        rospy.sleep(1)

    def test_scene(self):
        with generate_silence() as fname:
            content_url = 'file://{}'.format(fname)
            scene = {
                'name': NAME,
                'description': NAME,
                'duration': 60,
                'slug': NAME,
                'windows': [
                    {
                        'activity': 'video',
                        'activity_config': {
                        },
                        'assets': [
                            content_url,
                        ],
                        'presentation_viewport': 'center',
                        'width': 640,
                        'height': 480,
                        'x_coord': 0,
                        'y_coord': 0,
                    },
                ],
            }
            self.scene_pub.publish(GenericMessage(type='json', message=json.dumps(scene)))
            rospy.sleep(1)

            subprocess.check_call([
                'pgrep',
                '-af',
                'gst_video_sync\s.*-u\s{}'.format(content_url),
            ])

            scene = {
                'name': NAME + '_off',
                'description': NAME + '_off',
                'duration': 0,
                'slug': NAME + '_off',
                'windows': [],
            }
            self.scene_pub.publish(GenericMessage(type='json', message=json.dumps(scene)))
            rospy.sleep(1)

            with self.assertRaises(subprocess.CalledProcessError):
                subprocess.check_call([
                    'pgrep',
                    '-af',
                    'gst_video_sync\s.*-u\s{}'.format(content_url),
                ])


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestGstVideoSyncIntegration)
