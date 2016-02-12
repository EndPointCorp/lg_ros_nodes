from lg_chrome_control import PageChecker
from subprocess import Popen
from urllib2 import urlopen
from time import time

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
            active_pages = urlopen('http://localhost:%s' % self.port, timeout=1)
        except Exception:
            return
        page_checker = PageChecker(active_pages.read())
        print 'monitoring %s ...' % i; i += 1
        for page in self.allowed_pages:
            # if we find a match, just return
            if page_checker.check_pages(page) == 0:
                return
        for page in self.bad_pages:
            if page_checker.check_pages(page) == 0:
                print 'relaunching...'
                self.relaunch()
                self.last_relaunch = time()
                print 'relaunched...'
                return
        print 'killing chrome...'
        self.pkill_chrome()
        print 'killed chrome...'

    def relaunch(self):
        p = Popen(['lg-relaunch'])
        p.wait()

    def pkill_chrome(self):
        p = Popen(['pkill', 'chrome'])
        p.wait()
