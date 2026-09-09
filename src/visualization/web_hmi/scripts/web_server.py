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

    # 개발용 정적 서버: HTML/JSX 를 고치면 즉시 반영돼야 한다. 기본
    # SimpleHTTPRequestHandler 는 Cache-Control 을 안 붙여서 브라우저가
    # 휴리스틱 캐싱으로 구 index_*.html 을 재사용할 수 있는데, 그러면
    # 새로 추가된 <script> 태그가 빠진 채 새 JSX 만 로드돼 React 가
    # "element type is invalid" 로 트리 전체를 못 그린다 (지도까지 사라짐).
    class NoCacheHandler(SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_header('Cache-Control', 'no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            SimpleHTTPRequestHandler.end_headers(self)

    handler = partial(NoCacheHandler, directory=web_dir)
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
