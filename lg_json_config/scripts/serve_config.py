#!/usr/bin/env python3

import os
import rospy
import urllib.request, urllib.error, urllib.parse
import json
from lg_msg_defs.srv import JSONConfig
from lg_msg_defs.srv import JSONConfigMore
from lg_msg_defs.srv import JSONConfigResponse
from lg_msg_defs.srv import JSONConfigMoreResponse
from lg_common.helpers import run_with_influx_exception_handler


NODE_NAME = 'lg_json_config'
SRV_QUERY = '/'.join(('', NODE_NAME, 'query'))
SRV_MORE_QUERY = '/'.join(('', NODE_NAME, 'query_more'))
PARAM_URL = '~url'
DEFAULT_URL = 'http://lg-head/portal/config.json'


class ConfigRequestHandler():
    def get_url(self):
        return rospy.get_param(PARAM_URL, DEFAULT_URL)

    def get_config(self, url=None):
        if not url:
            url = self.get_url()
        try:
            response = urllib.request.urlopen(url)
            r = response.read()
        except Exception as e:
            r = """{"result": "error": "reason": "%s"}""" % e
            return r

        try:
            json.loads(r)
        except ValueError:
            r = """{"result": "error": "reason": "Could not parse json from %s"}""" % url
            return r

        return r

    def handle_request(self, request):
        config = self.get_config()
        return JSONConfigResponse(config)

    def handle_request_more(self, request):
        # using the default url to avoid a security hole where a user could
        # input file:// as the ~url param, and get access to any file on the
        # local filesystem
        url = os.path.join(os.path.dirname(DEFAULT_URL), request.filename)
        config = self.get_config(url)
        return JSONConfigMoreResponse(config)


def main():
    rospy.init_node(NODE_NAME)

    handler = ConfigRequestHandler()

    rospy.Service(
        SRV_QUERY,
        JSONConfig,
        handler.handle_request
    )

    rospy.Service(
        SRV_MORE_QUERY,
        JSONConfigMore,
        handler.handle_request_more
    )

    rospy.spin()


if __name__ == '__main__':
    run_with_influx_exception_handler(main, NODE_NAME)
