cmake_minimum_required(VERSION 3.0.2)
project(control_panel)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mmc_msgs
  rviz
)

# Qt5 찾기
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Catkin 패키지 설정
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs mmc_msgs rviz
)

# Include 디렉토리
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Qt 설정 - AUTOMOC를 먼저 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 헤더 파일 명시
set(HEADER_FILES
  include/control_panel/mode_panel.h
)

# 소스 파일
set(SOURCE_FILES
  src/mode_panel.cpp
)

# 라이브러리 생성
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

# 라이브러리 링크
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

# 의존성 추가 (중요!)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 설치 규칙
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)