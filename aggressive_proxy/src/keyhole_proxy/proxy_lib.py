class MainHandler(tornado.web.RequestHandler):
    def set_default_headers(self):
        self.set_header('Access-Control-Allow-Origin', '*')

    def get(self, args=None):
        url = self.get_argument('url', None, True)
        if url is None:
            rospy.logerr('Error with url')
            self.write('Error with url')
            return
        proxy_item = urllib.urlopen(url)
        self.write(proxy_item.read())
