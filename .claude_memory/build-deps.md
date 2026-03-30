# Build Dependencies

## System Packages (apt)
- `ros-noetic-nmea-msgs` - ublox_gps 패키지 의존성
- `ros-noetic-jsk-rviz-plugins` - OverlayText (stat_display, can 패키지)
- `kvaser-canlib-dev` (5.38.841) - Kvaser CAN library (pre-installed via PPA)
- `kvaser-linlib-dev` - Kvaser LIN library
- `kvaser-drivers-dkms` - Kvaser kernel drivers
- `python3-geopandas`, `python3-shapely` - Shapefile 처리용

## Kvaser SDK (수동 설치)
- `kvlibsdk_5_38_841.tar.gz` - kvaDbLib, kvlclib, kvmlib 포함
- Download: https://pim.kvaser.com/var/assets/Product_Resources/7330130981966/5.38.841/kvlibsdk_5_38_841.tar.gz
- Install: `cd /tmp/kvlibsdk && make && sudo make install`
- Provides: `/usr/lib/libkvadblib.so`, `/usr/include/kvaDbLib.h`
- canlib-dev 버전과 맞춰서 설치해야 함 (현재 5.38.841)

## CMake Notes
- `rviz_filter` 패키지: `proj` 라이브러리 못 찾는 warning (동작에 영향 없음)
- `system_lib` DEPENDS warning: 여러 diagnostic 패키지 (동작에 영향 없음)
- `can/CMakeLists.txt`: `katech_custom_msgs` 의존성 제거됨 (사용자 수정)
