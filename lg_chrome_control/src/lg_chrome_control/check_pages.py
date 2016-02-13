#!/usr/bin/env python
# vim:set ai sts=4 ts=4 sw=4 expandtab filetype=python:

from optparse import OptionParser
import rospy
import sys,json


def opts():
    parser = OptionParser()
    parser.add_option("--debug", dest="debugging", default=False)
    parser.add_option("--page-name", dest="page_name")
    return parser.parse_args()


class PageChecker(object):
    def __init__(self, pages, page_name=None, debug=False):
        try:
            self.pages = json.loads(pages)
        except ValueError:
            self.pages = []
            rospy.logerr('bad pages...')
            return

        self.page_name = page_name
        self.debug = debug

    def debug_print(self, msg):
        rospy.loginfo(msg)
        if self.debug:
            rospy.logdebug(msg)

    def check_pages(self, page_name=None):
        if page_name is None:
            page_name = self.page_name
        exit_val = 1
        for page in self.pages:
            if page_name in page["title"]:
                self.debug_print("found page with name: %s" % page["title"])
                exit_val = 0
        return exit_val

if __name__ == '__main__':
    options, _ = opts()
    if not options.page_name:
        raise Exception("No page name specified, use --page-name in arguments")
    checker = PageChecker(sys.stdin, options.page_name, options.debugging)
    exit_val = checker.check_pages()
    sys.exit(exit_val)
