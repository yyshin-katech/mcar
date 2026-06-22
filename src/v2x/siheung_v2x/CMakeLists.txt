cmake_minimum_required(VERSION 3.0.2)
project(siheung_v2x)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  j3224_msgs
  v2x_msgs
  mmc_msgs
  novatel_gps_msgs
  katech_custom_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs j3224_msgs v2x_msgs mmc_msgs novatel_gps_msgs katech_custom_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# link lib dir
link_directories(
  ${PROJECT_SOURCE_DIR}/lib/x86_64
)

## siheung_v2x_node: SPaT(KSR1600) + SDSM(2020)
## sdsm_decode.cpp → ffasn1-j2735-2020 (별도 번역 단위, 헤더 충돌 방지)
## j2735_decode.cpp + KSR1600.c → ffasn1-j2735-2026-KSR1600
add_executable(${PROJECT_NAME}_node
  src/j2735_decode.cpp
  src/sdsm_decode.cpp
  src/tim_publish.cpp
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

add_executable(j2735_universal_node src/j2735_universal_decoder.cpp)
target_link_libraries(j2735_universal_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)

## ── J2735 TIM RX ───────────────────────────────────────────
## OBU UDP packet -> J2735 UPER decode -> v2x_go_ahead/pedes_assist topics
add_executable(j2735_TIM_rx_node
  src/j2735_TIM_rx_node.cpp
  src/tim_publish.cpp
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(j2735_TIM_rx_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)
add_dependencies(j2735_TIM_rx_node ${catkin_EXPORTED_TARGETS})

add_executable(tim_test src/tim_decode_test.cpp)
target_link_libraries(tim_test
    ${catkin_LIBRARIES}
    ffasn1-base
    ffasn1-j2735-2020
)

## ── BSM TX ────────────────────────────────────────────────
## bsm_tx_node: BSM 송신. KSR1600 헤더 + 2020.so 링크
## (j2735_decode.cpp 와 같은 헤더지만 별도 executable 이라 충돌 없음)
add_executable(bsm_tx_node src/bsm_tx_node.cpp)
target_link_libraries(bsm_tx_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)

## ── MQTT SPaT RX ────────────────────────────────────────────
## mqtt_spat_rx_node: MQTT broker(V2N)에서 SPaT 수신
find_library(MOSQUITTO_LIB mosquitto)
if(NOT MOSQUITTO_LIB)
  message(FATAL_ERROR "libmosquitto not found. Install: sudo apt install libmosquitto-dev")
endif()

add_executable(mqtt_spat_rx_node
  src/mqtt_spat_rx_node.cpp
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(mqtt_spat_rx_node
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIB}
  ffasn1-base
  ffasn1-j2735-2020
)

## ── MQTT BSM TX ────────────────────────────────────────────
## mqtt_bsm_tx_node: MQTT broker(V2N)로 BSM publish
## (bsm_tx_node 의 BSM 인코딩 로직을 file-local 복제, libmosquitto 링크)
add_executable(mqtt_bsm_tx_node src/mqtt_bsm_tx_node.cpp)
target_link_libraries(mqtt_bsm_tx_node
  ${catkin_LIBRARIES}
  ${MOSQUITTO_LIB}
  ffasn1-base
  ffasn1-j2735-2020
)

## spat_merge_node: OBU + MQTT SPaT 를 교차로(IID) 단위 OBU 우선으로 병합 → /spat_merged
add_executable(spat_merge_node src/spat_merge_node.cpp)
target_link_libraries(spat_merge_node
  ${catkin_LIBRARIES}
)
add_dependencies(spat_merge_node ${catkin_EXPORTED_TARGETS})
