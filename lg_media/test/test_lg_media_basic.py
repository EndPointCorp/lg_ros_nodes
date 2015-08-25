#!/usr/bin/env python

"""
Ad hoc media application manager tests.

run tests on an individual package:
    rostest lg_media/launch/dev.launch

run entire lg_ros_notes test suite:
    catkin_make run_tests
    catkin_test_results build/test_results --all --verbose

"""


import os
import unittest
import time
import json

import pytest
import rostest
import rospy
import rospkg
import rostopic
import rosgraph.masterapi

from lg_media.msg import AdhocMedia
from lg_media.msg import AdhocMedias
from lg_media.srv import MediaAppsInfo
from lg_media.srv import MediaAppsInfoResponse
from lg_media import SRV_QUERY
from lg_media import AppInstance


class MockProc(object):
    def __init__(self, cmd, state):
        self.cmd = cmd
        self.state = state


class TestAppInstance(object):
    def test_basic(self):
        proc = MockProc("/command/path", "my_state")
        ai = AppInstance(proc, "/some/path", "/some/url")
        assert str(ai).find("FIFO='/some/path'") > 1


#class TestMediaService(unittest.TestCase):
class TestMediaService(object):

    @classmethod
    def setup_class(cls):
        pass

    @classmethod
    def teardown_class(cls):
        # TODO
        # do shutdown as seen in rostest.rosrun
        # assert ros services are not run anymore
        # it doesn't seem to be necessary if tests are run by means of the
        # entire package launch file
        #rospy.signal_shutdown('test complete')
        pass

    def setup_method(self, method):
        # print "setup_method, running '%s' ..." % method.__name__
        # nothing shall run at the beginning of a test case
        data = self.lg_media_service_call()
        assert data == {}

    def teardown_method(self, _):
        pass

    def shutdown_check_clean_up(self):
        """
        Send an empty media message which shuts everything running down.
        Check the tracked applications are gone and the previously
        existing FIFO files are cleaned up.

        """
        prev_data = self.lg_media_service_call()
        pub = rospy.Publisher("/media_service/left_one", AdhocMedias, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        pub.publish(AdhocMedias(medias=[]))
        count = 0
        while count < 10:
            time.sleep(1)
            data = self.lg_media_service_call()
            if data == {}:
                break
        else:
            pytest.fail("Test did not properly shutdown applications.")

        for app_info in prev_data.values():
            fifo = app_info.split(' ')[1]
            fifo.replace("FIFO='", '')
            fifo.replace("'", '')  # the trailing '
            assert not os.path.exists(fifo)

    def lg_media_service_call(self):
        """
        Perform rosservice call /lg_media/query
        and return data in dict.

        """
        rospy.wait_for_service(SRV_QUERY)
        proxy = rospy.ServiceProxy(SRV_QUERY, MediaAppsInfo)
        r = proxy()
        assert isinstance(r, MediaAppsInfoResponse)
        data = json.loads(r.json)
        return data

    def test_lg_media_service_call(self):
        """
        Assert on presence of service, call the service.

        """
        data = self.lg_media_service_call()
        assert data == {}

    def test_lg_media_topic_presence(self):
        """
        Check lg_media topic is there.

        """
        #master = rosgraph.masterapi.Master("caller")
        #topics = master.getTopicTypes()
        #assert ["/media_service/left_one", "lg_media/AdhocMedias"] in topics
        topic = "/media_service/left_one"
        topic_type, real_topic, msg_eval = rostopic.get_topic_type(topic, blocking=False)
        assert topic_type is not None, "Topic not found: {}".format(topic)
        assert topic_type == "lg_media/AdhocMedias"

    def get_media_msg(self, msg_id="1"):
        """
        Returns 2 kinds of media messages, according to the
        desired msg id which, after starting the application gets translated
        into tracking app id.

        """
        if msg_id == "1":
            media = AdhocMedia(id="1", url="https://zdenek.endpoint.com/kaiser_labus-7_statecnych.flv")
        if msg_id == "2":
            media = AdhocMedia(id="2", url="https://zdenek.endpoint.com/a_conference_call_in_real_life.mp4")
        media.geometry.x = 640
        media.geometry.y = 480
        media.geometry.width = 0
        media.geometry.height = 0
        return media

    def perform_test(self, wait_iterations=10, time_sleep=2, len_data=1, media_msg=None):
        """
        Perform a test on a message and application start oriented test case.

        """
        count = 0
        while count < wait_iterations:
            time.sleep(time_sleep)
            data = self.lg_media_service_call()
            try:
                assert len(data) == len_data
                assert media_msg.id in data.keys()
                assert data[media_msg.id].find(media_msg.url) > 1
                break
            except AssertionError:
                pass
            count += 1
        else:
            pytest.fail("mplayer doesn't seem to start on "
                        "media: %s, app status: %s" % (media_msg, data))

    def test_lg_media_start_app_request(self):
        """
        Emit onto a topic mplayer app start request and check
        internal state of the MediaService.

        """
        pub = rospy.Publisher("/media_service/left_one", AdhocMedias, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        media = self.get_media_msg(msg_id="1")
        pub.publish(AdhocMedias(medias=[media]))
        self.perform_test(len_data=1, media_msg=media)
        self.shutdown_check_clean_up()

    def test_lg_media_start_app_and_update_it(self):
        """
        Have mplayer application started. Then under the same ID update the
        source URL and check the media manager internal status.

        """
        pub = rospy.Publisher("/media_service/left_one", AdhocMedias, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        media = self.get_media_msg(msg_id="1")
        pub.publish(AdhocMedias(medias=[media]))
        self.perform_test(len_data=1, media_msg=media)
        # update
        media = self.get_media_msg(msg_id="2")
        # got different media message now, but publish it under the previously used id
        media.id = "1"
        pub.publish(AdhocMedias(medias=[media]))
        self.perform_test(len_data=1, media_msg=media)
        self.shutdown_check_clean_up()

    def test_lg_media_start_app_and_start_another_one(self):
        """
        Have mplayer application started. Then emit a message with different
        app id than the one which is already running.
        The running one should be shutdown and the new one started.

        """
        pub = rospy.Publisher("/media_service/left_one", AdhocMedias, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        media1 = self.get_media_msg(msg_id="1")
        pub.publish(AdhocMedias(medias=[media1]))
        self.perform_test(len_data=1, media_msg=media1)
        # trigger another message, another application
        media2 = self.get_media_msg(msg_id="2")
        pub.publish(AdhocMedias(medias=[media2]))
        # there should remain just 1 application, so len_data=1
        # the previous app should be closed now, hence untracked
        # and only the recent app request running
        self.perform_test(len_data=1, media_msg=media2)
        self.shutdown_check_clean_up()

    def test_lg_media_start_two_apps_at_once(self):
        """
        Start two mplayer applications at once.

        """
        pub = rospy.Publisher("/media_service/left_one", AdhocMedias, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        media1 = self.get_media_msg(msg_id="1")
        media2 = self.get_media_msg(msg_id="2")
        pub.publish(AdhocMedias(medias=[media1, media2]))
        count = 0
        while count < 10:
            time.sleep(2)
            data = self.lg_media_service_call()
            try:
                assert len(data) == 2
                assert media1.id in data.keys()
                assert media2.id in data.keys()
                assert data[media1.id].find(media1.url) > 1
                assert data[media2.id].find(media2.url) > 1
                break
            except AssertionError:
                pass
            count += 1
        else:
            pytest.fail("mplayer doesn't seem to start on "
                        "media1: %s, media2: %s, app status: %s" % (media1, media2, data))

        self.shutdown_check_clean_up()


if __name__ == "__main__":
    # unittest test
    # test class must inherit from unittest.TestCase, not from object
    #rostest.rosrun("lg_media", "test_lg_media_basic", TestMediaService)
    #import sys
    #sys.exit(0)

    # pytest must provide result XML file just as rostest.rosrun would do
    # otherwise: FAILURE: test [test_lg_media_basic] did not generate test results

    test_pkg = "lg_media"
    test_name = "test_lg_media_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    test_path = os.path.abspath(os.path.dirname(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    pytest.main("%s -s -v --junit-xml=%s" % (test_path, pytest_result_path))
