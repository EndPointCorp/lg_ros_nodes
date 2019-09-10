#!/usr/bin/env python3

"""
lg offliner basic offline tests (not requiring the ROS node itself to run).

"""


import os

import pytest
import rospkg

from lg_offliner import ROS_NODE_NAME
from lg_offliner import ConnectivityResults
from lg_offliner import Checker


class TestConnectivityResults(object):

    def test_instantiation(self):
        cr = ConnectivityResults(max_num_of_rounds_to_retain=50,
                                 num_of_last_check_rounds_consider=4)
        assert isinstance(cr.data, list)
        assert len(cr.data) == 0
        assert cr.max_num_of_rounds_to_retain == 50
        assert cr.num_of_last_check_rounds_consider == 4

    def test_add(self):
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10)
        for i in range(15):
            cr.add(i)
        assert cr.max_num_of_rounds_to_retain == 10
        assert len(cr.data) == 10
        # check first and last item current items
        assert cr.data[0] == 5
        assert cr.data[-1] == 14

    def test_am_i_offline(self):
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=2)
        cr.add(dict(cmd1=0, cmd2=0))
        cr.add(dict(cmd1=0, cmd2=0))
        assert not cr.am_i_offline()
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=1)
        cr.add(dict(cmd1=0, cmd2=0))
        cr.add(dict(cmd1=0, cmd2=0))
        assert not cr.am_i_offline()
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=1)
        cr.add(dict(cmd1=0, cmd2=1))
        assert not cr.am_i_offline()
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=1)
        cr.add(dict(cmd1=1, cmd2=1))
        assert cr.am_i_offline()
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=2)
        cr.add(dict(cmd1=1, cmd2=1))
        cr.add(dict(cmd1=1, cmd2=1))
        assert cr.am_i_offline()
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=2)
        cr.add(dict(cmd1=1, cmd2=1))
        cr.add(dict(cmd1=1, cmd2=0))
        assert not cr.am_i_offline()
        # take into consideration more items than actually exits
        # handled OK by negative index in the method
        cr = ConnectivityResults(max_num_of_rounds_to_retain=10,
                                 num_of_last_check_rounds_consider=2)
        cr.add(dict(cmd1=1, cmd2=0))
        assert not cr.am_i_offline()


if __name__ == "__main__":
    test_pkg = ROS_NODE_NAME
    test_name = "test_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    #pytest.main("-vv -rfsX -s --junit-xml=%s --cov=. --cov-report term-missing %s" % (pytest_result_path, test_path))
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
