# rs_sdk_user changelog

**Date: 2022-08-28**

**Version: v3.1-r0.4.31**

Feat:
1. add receive demo of communication module. Support native/native_bytes_3_0/native_bytes_3_1/proto/ros/ros2 protocol.

Fix:
1. fix bug of ROI ID in V2R protocol
2. fix bug of synchronization of multiple Point Cloud.
3. fix bug of localization module in Pseries strategy.

**Date: 2022-08-10**

**Version: v3.1-r0.4.30**

Fix: fix bug of ROI display

**Date: 2022-08-03**

**Version: v3.1-r0.4.29**

Fix: fix bug of V2R protocol.

**Date: 2022-08-01**

**Version: v3.1-r0.4.28**

Fix:
1. Fix the code compilation problem that the communication module uses proto to send location information.
2. Fixed frequent software crashes under the DTGH V1.2 protocol.

**Date: 2022-07-27**

**Version: v3.1-r0.4.27**

Fix: fix bug of message header of 3.1 bytes method in communication module.

**Date: 2022-07-13**

**Version: v3.1-r0.4.26**

Fix: fix bug of compiling proto code.

**Date: 2022-06-20**

**Version: v3.1-r0.4.25**

Fix: fix bug of the timestamp in DTGH v1.2

**Date: 2022-04-01**

**Version: v3.1-r0.4.24**

Fix: fix the bug of box corner filter of roi is invalid.
Feat: add some comment of the interface of function. 

**Date: 2022-03-28**

**Version: v3.1-r0.4.23**

Fix: Fixed the bug that in Pseries and V2r, the trajectory/history_velocity/history_type of object was empty.

**Date: 2022-03-25**

**Version: v3.1-r0.4.22**

Fix: Fix bug that there is an empty polygon in object info.

**Date: 2022-03-11**

**Version: v3.1-r0.4.21**

Fix: Fix bug of auto_align compilation failure.

**Date: 2022-03-08**

**Version: v3.1-r0.4.20**

Fix: fix bug of perception range display.

**Date: 2022-03-01**

**Version: v3.1-r0.4.19**

Fix: fix localization bug
Feat: ground points use ground_filter and ai

**Date: 2022-02-25**

**Version: v3.1-r0.4.18**

Fix: fix bug of roi display.

**Date: 2022-02-16**

**Version: v3.1-r0.4.17**

Fix: fix bug of semantic value of the point in ai module.

**Date: 2022-01-28**

**Version: v3.1-r0.4.16**

Feat: update x86_64 libs, add notes of config, improve ROI function.

Fix: fix bug of RVIZ display.

**Date: 2022-01-25**

**Version: v3.1-r0.4.15**

Feat: update x86_64 libs, default adapt cuda10.x

**Date: 2022-01-24**

**Version: v3.1-r0.4.14**

Feat: update x86_64 libs, adapt ubuntu16.04

**Date: 2022-01-21**

**Version: v3.1-r0.4.13**

Feat: auto_align support ROS2 environment, update communication_user_guide doc.

**Date: 2022-01-07**

**Version: v3.1-r0.4.12**

Feat: fix timestamp bug, add tracking for bsd obstacle, update docs

**Date: 2021-12-21**

**Version: v3.1-r0.4.11**

Feat: add proto ,native bytes ,and ros2 communication

**Date: 2021-12-14**

**Version: v3.1-r0.4.10**

Feat: update aarch64 libs

**Date: 2021-12-08**

**Version: v3.1-r0.4.9**

Feat: support fusion lidar send, custom log path

**Date: 2021-12-07**

**Version: v3.1-r0.4.8**

Feat: add ros packet output

**Date: 2021-11-25**

**Version: v3.1-r0.4.7**

Feat: update aarch64 libs

**Date: 2021-11-17**

**Version: v3.1-r0.4.6**

Feat: add mirror detection, add v2r websocket communication

**Date: 2021-10-26**

**Version: v3.1-r0.4.5**

Feat: add sensor offline check, add external pose support 

**Date: 2021-09-18**

**Version: v3.1-r0.4.4**

Fix: fix pcap error , update docs 

**Date: 2021-09-16**

**Version: v3.1-r0.4.3**

Feat: update localization , driver support pcap

**Date: 2021-09-10**

**Version: v3.1-r0.4.2**

Feat: add other sensors calib params

**Date: 2021-09-03**

**Version: v3.1-r0.4.1**

Feat: add fov config for sequence fusion

**Date: 2021-08-24**

**Version: v3.1-r0.4.0**

Feat: support localization

