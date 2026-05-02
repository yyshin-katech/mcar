#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Open the web HMI in the default browser. Best-effort, fault-tolerant."""
import shutil
import subprocess
import sys
import time

import rospy


def main():
    rospy.init_node('web_hmi_open_browser', anonymous=True)
    port = int(rospy.get_param('~port', 8088))
    page = str(rospy.get_param('~page', 'index.html')).lstrip('/')
    url = f"http://localhost:{port}/{page}" if page else f"http://localhost:{port}"

    # Wait briefly so the http.server is listening.
    time.sleep(1.5)

    for opener in ("xdg-open", "explorer.exe", "open"):
        path = shutil.which(opener)
        if not path:
            continue
        try:
            subprocess.Popen([path, url],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            rospy.loginfo("open_browser: launched %s %s", opener, url)
            return
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("open_browser: %s failed: %s", opener, e)

    rospy.logwarn("open_browser: no opener found; visit %s manually", url)


if __name__ == '__main__':
    main()
