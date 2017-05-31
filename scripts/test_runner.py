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
    g_tests = False
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
            res = re.search("^[^#]*catkin_add_gtest\(.*\)", l)
            if res:
                g_tests = True
    return nose_tests, ros_tests, g_tests


def get_tests():
    cmakes = get_cmakes()
    nose_tests = []
    ros_tests = []
    g_tests = []
    for cmake in cmakes:
        nose, ros, gt = lookup(cmake + '/CMakeLists.txt')
        nose_tests += [cmake + '/' + test for test in nose]
        ros_tests += [cmake + '/' + test for test in ros]
        if gt:
            g_tests.append(os.path.basename(cmake))
    return nose_tests, ros_tests, g_tests


def pep8_test():
    ret = os.system('pep8 --exclude=rosbridge*,appctl,wiimote,rosout_logger,lg_cms_director,docker_nodes --config=./setup.cfg catkin/src/*/')
    return ret


def cppcheck_test():
    ret = os.system("cppcheck -icatkin/src/wiimote --enable=style --error-exitcode=1 --suppressions-list=cppcheck_suppressions.txt catkin/src")
    return ret


def gjslint_test():
    ret = os.system("gjslint --nojsdoc --max_line_length 120 --disable 0001,0002,0131,0120 -e 'lib,lg_cms_director,panovideosync' -r .")
    return ret


def run_tests():
    nose_tests, ros_tests, g_tests = get_tests()
    fail_flags = {}
    for nose_test in nose_tests:
        # rosunit will urn the offline test just the same
        # benefit is that it respects pytest stuff
        # previous, nosetests command was this:
        # c = 'nosetests --verbosity=3 -s -l DEBUG %s' % nose_test
        c = "rosunit %s" % nose_test
        print "RUNNING: '%s'" % c
        ret = os.system(c)
        fail_flags[nose_test] = ret
    for ros_test in ros_tests:
        c = 'rostest %s' % ros_test
        print "RUNNING: '%s'" % c
        ret = os.system(c)
        fail_flags[ros_test] = ret
    for g_test in g_tests:
        c = 'cd catkin; catkin_make run_tests_%s_gtest' % g_test
        print "RUNNING: '%s'" % c
        ret = os.system(c)
        fail_flags[g_test + '_gtest'] = ret
    c = 'coveralls'
    os.system(c)
    fail_flags['pep8'] = pep8_test()
    fail_flags['cppcheck'] = cppcheck_test()
    fail_flags['gjslint'] = gjslint_test()
    print "\n\nFINAL SUMMARY:\n"
    for test, flag in sorted(fail_flags.items()):
        print "RAN TEST: %s\nGot exit code %d" % (test, flag)
    # check for non-zero exit status, and fail if found
    if filter(None, fail_flags.values()):
        sys.exit(FAIL)

if __name__ == '__main__':
    run_tests()
