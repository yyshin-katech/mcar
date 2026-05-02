#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tiny HTTP server for the web_hmi static assets.

Serves the package's ``web/`` directory on ``~port`` (default 8088).
Wraps ``http.server`` so it integrates as a ``<node>`` in the launch file.
"""
import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from functools import partial

import rospy


def main():
    rospy.init_node('web_hmi_server', anonymous=False)

    port = int(rospy.get_param('~port', 8088))
    web_dir = rospy.get_param('~web_dir', None)
    page = str(rospy.get_param('~page', '')).lstrip('/')

    if not web_dir:
        # Default to the sibling web/ directory of this script.
        here = os.path.dirname(os.path.realpath(__file__))
        web_dir = os.path.normpath(os.path.join(here, '..', 'web'))

    if not os.path.isdir(web_dir):
        rospy.logfatal("web_hmi_server: web_dir does not exist: %s", web_dir)
        sys.exit(2)

    handler = partial(SimpleHTTPRequestHandler, directory=web_dir)
    server = ThreadingHTTPServer(('0.0.0.0', port), handler)

    url = "http://localhost:%d/%s" % (port, page) if page else "http://localhost:%d" % port
    rospy.loginfo("web_hmi_server: serving %s on http://0.0.0.0:%d",
                  web_dir, port)
    rospy.loginfo("web_hmi_server: open this URL in a browser → %s", url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
