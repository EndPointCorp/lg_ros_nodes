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
from subprocess import CalledProcessError, check_output, STDOUT
from os import listdir
from os.path import isfile, join

PRINT_ROSTEST_LOGS = True
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
        raise Exception('Must run this script from the project root directory'
                        ' or inside the scripts dir')


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
    ret = os.system('pep8 --exclude=rosbridge*,catkin,lg_cms_director,docker_nodes')
    return ret


def run_command(command):
    """
    Execute comand and prints the stdout
    """
    print "RUNNING: '%s'" % command
    return os.system(command)


def run_tests():
    nose_tests, ros_tests = get_tests()
    fail_flags = {}
    for nose_test in nose_tests:
        # rosunit will urn the offline test just the same
        # benefit is that it respects pytest stuff
        # previous, nosetests command was this:
        # c = 'nosetests --verbosity=3 -s -l DEBUG %s' % nose_test
        c = "rosunit %s" % nose_test
        fail_flags[nose_test] = run_command(c)
    for ros_test in ros_tests:
        c = 'rostest %s' % ros_test
        print "RUNNING: '%s'" % c
        fail_flags[ros_test] = run_command(c)
    fail_flags['pep8'] = pep8_test()
    print "\n\nFINAL SUMMARY:\n"
    for test, flag in sorted(fail_flags.items()):
        print "RAN TEST: %s\nGot exit code %d" % (test, flag)
    # check for non-zero exit status, and fail if found
    if filter(None, fail_flags.values()):
        if PRINT_ROSTEST_LOGS:
            print "========== Some tests are failed, print logs =================="
            logs_dir = "/home/test_docker/.ros/log"
            log_files = [f for f in listdir(logs_dir) if isfile(join(logs_dir, f))]
            for f in log_files:
                print "Print file %s:" % join(logs_dir, f)
                with open(join(logs_dir, f), 'r') as fin:
                    print fin.read()
            print "---------------------------------------------------------------"
        sys.exit(FAIL)

if __name__ == '__main__':
    run_tests()
