#!/usr/bin/env python3
"""
Runs the test suite and the linters, and exits non-zero if any of them failed.

This is what CI invokes, via scripts/run_tests.sh.

catkin runs the tests: `catkin_make run_tests` executes every test registered
in a package's CMakeLists.txt (catkin_add_nosetests and add_rostest), and
`catkin_test_results` is what reports the failures, since run_tests itself
exits 0 whatever happens. Coverage and the three linters run afterwards, and
every exit code is collected so one failure does not hide the rest.
"""

import os
import sys

FAIL = 1


def pep8_test():
    ret = os.system('pycodestyle --config=./setup.cfg .')
    return ret


def cppcheck_test():
    ret = os.system('cppcheck -icatkin -iwiimote -iuWebSockets --enable=style --error-exitcode=1 --suppressions-list=cppcheck_suppressions.txt .')
    return ret


def jslint_test():
    ret = os.system("eslint .")
    return ret


def run_tests():
    fail_flags = {}
    os.system('cd catkin; catkin_make run_tests -DNOSETESTS=/usr/bin/nosetests3')
    fail_flags['catkin'] = os.system('cd catkin; catkin_test_results')
    # Best effort: coverage reporting should not decide whether the build passes.
    os.system('coveralls')
    fail_flags['pep8'] = pep8_test()
    fail_flags['cppcheck'] = cppcheck_test()
    fail_flags['jslint'] = jslint_test()
    print("\n\nFINAL SUMMARY:\n")
    for test, flag in sorted(fail_flags.items()):
        print("RAN TEST: %s\nGot exit code %d" % (test, flag))
    # check for non-zero exit status, and fail if found
    if [_f for _f in list(fail_flags.values()) if _f]:
        sys.exit(FAIL)


if __name__ == '__main__':
    run_tests()
