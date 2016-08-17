import rospy

import tornado.web
from tornado.httputil import HTTPHeaders
import urllib


class ProxyHandler(tornado.web.RequestHandler):
    def initialize(self, client, upstream_socket):
        # Reference the single AsyncHTTPClient instance.
        self.client = client
        self.upstream_socket = upstream_socket

    def build_client_headers(self):
        client_headers = HTTPHeaders(self.request.headers)
        #client_headers['Host'] = 'kh.google.com'
        #if 'GoogleEarth' not in client_headers['User-Agent']:
        #    client_headers['User-Agent'] = 'GoogleEarth/7.1.2.2041(X11;Linux (3.13.0.0);en;kml:2.2;client:Free;type:default)'
        return client_headers

    def build_client_request(self):
        # https://github.com/tornadoweb/tornado/issues/1213
        if self.request.method in ("POST", "PUT", "PATCH"):
            client_request = tornado.httpclient.HTTPRequest(
                url='http://' + self.upstream_socket + self.request.uri,
                method=self.request.method,
                headers=self.build_client_headers(),
                body=self.request.body,
                request_timeout=10.0
            )
        else: # No bodies for GET or other HTTP requests.
            client_request = tornado.httpclient.HTTPRequest(
                url='http://' + self.upstream_socket + self.request.uri,
                method=self.request.method,
                headers=self.build_client_headers(),
                request_timeout=1.0
            )
        #rospy.loginfo(self.request.headers)
        #rospy.loginfo(client_request.headers)
        return client_request

    def response_callback(self, response):
        #TODO this function needs cleanup.
        if response.error:
            rospy.logerr('got error: %s.' % (response.error))
            if not hasattr(response.error, 'code'):
                rospy.logerr('no error.code')
                self.success_callback(response)
                return
            #http://www.tornadoweb.org/en/stable/httpclient.html#tornado.httpclient.HTTPError
            if int(response.error.code) == int(599):
                # RECURSION
                rospy.logerr('Trying again!')
                self.fetch_resource(self.response_callback)
            elif int(response.error.code) == 304: # Not changed.
                rospy.loginfo('got response: %s in %s seconds' % (response.code, response.request_time))
#                self.success_callback(response)
                self.set_status(response.code)
                self.finish()
            else:
                rospy.logerr('got error: %s %s %s not trying again.' % (type(response.error.code), response.error.code, response.error))
                self.success_callback(response)
        else:
            rospy.loginfo('got response: %s in %s seconds' % (response.code, response.request_time))
            self.success_callback(response)

    def success_callback(self, response):
        try:
            self.set_status(response.code)
        except ValueError:
            self.set_status(500)
        if response.body:
          self.write(response.body)
        # iterate over headers.
        for header, value in response.headers.iteritems():
            self.set_header(header, value)
        self.finish()

    def fetch_resource(self, callback):
        self.client.fetch(self.build_client_request(), callback)

    @tornado.web.asynchronous
    def get(self, args=None):
        self.fetch_resource(self.response_callback)

    @tornado.web.asynchronous
    def post(self, args=None):
        # Handle POST same as GET.
        self.fetch_resource(self.response_callback)
