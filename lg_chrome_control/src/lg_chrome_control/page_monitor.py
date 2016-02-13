from lg_chrome_control import PageChecker
from subprocess import Popen
from urllib2 import urlopen
from time import time
import rospy

class PageMonitor(object):
    
    def __init__(self, cmd_app='', cmd_bin='', allowed_pages=[],
                 bad_pages=[], port=9942, verbose=False):
        self.cmd_app = cmd_app
        self.cmd_bin = cmd_bin
        self.allowed_pages = allowed_pages
        self.bad_pages = bad_pages
        self.port = port
        self.verbose = verbose
        # records last relaunch so we don't run the page checker
        # within 10 seconds of our last relaunch
        self.last_relaunch = time()

    def monitor(self, *args, **kwargs):
        if time() - self.last_relaunch < 10:
            return True
        i = 0
        try:
            active_pages = urlopen('http://localhost:%s/json' % self.port, timeout=1)
        except Exception:
            return
        text = active_pages.read()
        page_checker = PageChecker(text, debug=True)
        for page in self.allowed_pages:
            # if we find a match, just return
            ret = page_checker.check_pages(page)
            rospy.loginfo('got ret (%s) for page (%s)' % (ret, page))
            if ret == 0:
                return

        # if we are here, the page hasn't been found...
        self.last_relaunch = time()
        for page in self.bad_pages:
            if page_checker.check_pages(page) == 0:
                rospy.loginfo('relaunching...')
                self.relaunch()
                return
        rospy.loginfo('killing chrome...')
        self.pkill_chrome()

    def relaunch(self):
        p = Popen(['lg-relaunch'])
        p.wait()

    def pkill_chrome(self):
        p = Popen(['pkill', 'chrome'])
        p.wait()
