#!/usr/bin/env python
"""
Custom script to run nosetests and rostests as defined in CMakeLists.txt
files.

catkin commands (catkin_make run_tests, catkin_make tests and
catkin_make tests_results) were troublesome, esp. in the jenkiins environment.
One was giving nice output but always returning 0 exit status (so jenkins was
still green), the other command was reliable on exit status but had 0
unhelpful output ... Hence this script runs directly rostests and nosetests
commands and checks exit status from them.

"""

import os
import re
import sys

FAIL = 1


def set_root_dir():
    """
    make sure we run this from the project root directory
    """
    root_dir = os.getcwd()

    if os.path.basename(root_dir) == 'scripts':
        os.chdir('..')
        root_dir = os.getcwd()
    if not os.path.isdir('.git'):
        raise Exception('Must run this script from the lg_ros_nodes project root'
                        ' directory or inside the scripts dir')


def get_cmakes():
    """
    Assumes you are in the project root directory, that will
    be handled elsewhere in this script
    """
    ret = []
    full_path = os.getcwd() + '/catkin/src'
    ls = os.listdir(full_path)
    dirs = [full_path + '/' + d for d in ls if os.path.isdir(full_path + '/' + d)]
    ret = [d for d in dirs if os.path.exists(d + "/CMakeLists.txt")]
    return ret


def lookup(path):
    ros_tests = []
    nose_tests = []
    with open(path) as f:
        for l in f:
            res = re.search("^[^#]*add_rostest\((.*)\)", l)
            if res:
                rostest_path = res.groups()[0]
                ros_tests.append(rostest_path)
            res = re.search("^[^#]*catkin_add_nosetests\((.*)\)", l)
            if res:
                nosetest_path = res.groups()[0]
                nose_tests.append(nosetest_path)
    return nose_tests, ros_tests


def get_tests():
    cmakes = get_cmakes()
    nose_tests = []
    ros_tests = []
    for cmake in cmakes:
        nose, ros = lookup(cmake + '/CMakeLists.txt')
        nose_tests += [cmake + '/' + test for test in nose]
        ros_tests += [cmake + '/' + test for test in ros]
    return nose_tests, ros_tests


def pep8_test():
    ret = os.system('pep8')
    ret += os.system('(cd lg_cms_director; pep8)')
    return ret


def run_tests():
    nose_tests, ros_tests = get_tests()
    fail_flags = {}
    for nose_test in nose_tests:
        ret = os.system('nosetests --verbosity=3 -s -l DEBUG %s' % nose_test)
        fail_flags[nose_test] = ret
    for ros_test in ros_tests:
        ret = os.system('rostest %s' % ros_test)
        fail_flags[ros_test] = ret
    fail_flags['pep8'] = pep8_test()
    for test, flag in fail_flags.items():
        print "RAN TEST: %s\nGot exit code %d" % (test, flag)
    # check for non-zero exit status, and fail if found
    if filter(None, fail_flags.values()):
        sys.exit(FAIL)

if __name__ == '__main__':
    run_tests()
