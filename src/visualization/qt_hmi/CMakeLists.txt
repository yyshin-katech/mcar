cmake_minimum_required(VERSION 3.0.2)
project(qt_hmi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt MOC/UIC/RCC.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mmc_msgs
)

# Qt6 first, then Qt5 fallback (Ubuntu 20.04 default = Qt5.12.8).
# M3 adds OpenGL widget module:
#   Qt6 → Qt6::OpenGLWidgets
#   Qt5 → Qt5::OpenGL  (QOpenGLWidget lives there in Qt5)
find_package(Qt6 6.2 COMPONENTS Core Gui Widgets OpenGLWidgets QUIET)
if (Qt6_FOUND)
  message(STATUS "qt_hmi: building against Qt6 (${Qt6_VERSION})")
  set(QT_HMI_QT_LIBS Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets)
else()
  find_package(Qt5 5.12 COMPONENTS Core Gui Widgets OpenGL QUIET)
  if (Qt5_FOUND)
    message(STATUS "qt_hmi: building against Qt5 (${Qt5_VERSION})")
    set(QT_HMI_QT_LIBS Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
  else()
    message(WARNING
      "qt_hmi: neither Qt6 nor Qt5 found — package will be skipped. "
      "Install qtbase5-dev + libqt5opengl5-dev (Ubuntu 20.04) "
      "or qt6-base-dev (22.04+).")
    catkin_package()
    return()
  endif()
endif()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mmc_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# M1 source set (skeleton + RosBridge + MainWindow shell).
set(QT_HMI_M1_SOURCES
  src/main.cpp
  src/RosBridge.cpp
  src/MainWindow.cpp
)

set(QT_HMI_M1_HEADERS
  include/qt_hmi/HmiTypes.h
  include/qt_hmi/RosBridge.h
  include/qt_hmi/MainWindow.h
)

# M2 source set (F1 dashboard + 7 small QPainter widgets).
set(QT_HMI_M2_SOURCES
  src/widgets/F1Sections.cpp
  src/widgets/F1Dashboard.cpp
)

set(QT_HMI_M2_HEADERS
  include/qt_hmi/style/F1Tokens.h
  include/qt_hmi/widgets/F1Sections.h
  include/qt_hmi/widgets/F1Dashboard.h
)

# M3 source set (MapScene QOpenGLWidget + LayerStyle table). Shaders are
# inline C-strings inside MapScene.cpp; no .qrc needed.
set(QT_HMI_M3_SOURCES
  src/widgets/MapScene.cpp
  src/style/LayerStyle.cpp
)

set(QT_HMI_M3_HEADERS
  include/qt_hmi/widgets/MapScene.h
  include/qt_hmi/style/LayerStyle.h
)

# M4 source set (ControlPanel right dock + V2X TrafficLightWidget). BagStrip
# / DRIVE MODE click handlers are scoped out of this milestone — F1Dashboard
# already paints the f1widgets::TrafficLight on the left panel section 03.
set(QT_HMI_M4_SOURCES
  src/widgets/ControlPanel.cpp
  src/widgets/TrafficLightWidget.cpp
)

set(QT_HMI_M4_HEADERS
  include/qt_hmi/widgets/ControlPanel.h
  include/qt_hmi/widgets/TrafficLightWidget.h
)

add_executable(qt_hmi_node
  ${QT_HMI_M1_SOURCES}
  ${QT_HMI_M1_HEADERS}
  ${QT_HMI_M2_SOURCES}
  ${QT_HMI_M2_HEADERS}
  ${QT_HMI_M3_SOURCES}
  ${QT_HMI_M3_HEADERS}
  ${QT_HMI_M4_SOURCES}
  ${QT_HMI_M4_HEADERS}
)

target_link_libraries(qt_hmi_node
  ${QT_HMI_QT_LIBS}
  ${catkin_LIBRARIES}
)

# mmc_msgs is a generated message package — make sure its headers are built
# before qt_hmi_node compiles RosBridge.cpp.
add_dependencies(qt_hmi_node ${catkin_EXPORTED_TARGETS})

install(TARGETS qt_hmi_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
