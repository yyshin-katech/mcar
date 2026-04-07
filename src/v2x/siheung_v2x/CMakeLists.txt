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
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs j3224_msgs v2x_msgs mmc_msgs
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
  src/ffasn1-j2735-2026-KSR1600.c
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)

add_executable(j2735_universal_node src/j2735_universal_decoder.cpp)
target_link_libraries(j2735_universal_node
  ${catkin_LIBRARIES}
  ffasn1-base
  ffasn1-j2735-2020
)

add_executable(tim_test src/tim_decode_test.cpp)
target_link_libraries(tim_test
    ${catkin_LIBRARIES}
    ffasn1-base
    ffasn1-j2735-2020
)
