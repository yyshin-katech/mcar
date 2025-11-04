cmake_minimum_required(VERSION 3.0.2)
project(can)

add_definitions(-std=c++14)
ADD_COMPILE_OPTIONS (-O3 -march=native)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mmc_msgs
  roslib
  message_generation
  v2x_msgs
  katech_diagnostic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs mmc_msgs roslib v2x_msgs katech_diagnostic_msgs

)

############################## Adding External Library
find_library(CANLIB_LIBRARY canlib /usr/include)
find_library(KVADBLIB_LIBRARY kvadblib /usr/include)
######################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} lib/can_pub_func.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############################################################################################# CAN_channel_initializer
add_executable(CAN_channel_initializer src/CAN_channel_initializer.cpp)
target_compile_options(CAN_channel_initializer PRIVATE)
add_dependencies(CAN_channel_initializer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CAN_channel_initializer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} CAN_channel_initializer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################################################################ percept_topic_only_front_lidar.cpp
add_executable(percept_topic_only_front_lidar src/percept_topic_only_front_lidar.cpp)
target_compile_options(percept_topic_only_front_lidar PRIVATE)
add_dependencies(percept_topic_only_front_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(percept_topic_only_front_lidar 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} percept_topic_only_front_lidar
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################ percept_topic_matcher.cpp
add_executable(percept_topic_matcher src/percept_topic_matcher.cpp)
target_compile_options(percept_topic_matcher PRIVATE)
add_dependencies(percept_topic_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(percept_topic_matcher 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} percept_topic_matcher
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################ cpt7_topic_matcher.cpp
add_executable(cpt7_topic_matcher src/cpt7_topic_matcher.cpp)
target_compile_options(cpt7_topic_matcher PRIVATE)
add_dependencies(cpt7_topic_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpt7_topic_matcher 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} cpt7_topic_matcher
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################################# RADAR CAN reader
add_executable(front_RADAR_CAN_reader src/front_RADAR_CAN_reader.cpp)
target_compile_options(front_RADAR_CAN_reader PRIVATE)
add_dependencies(front_RADAR_CAN_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_RADAR_CAN_reader 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} front_RADAR_CAN_reader
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################### vision CAN reader
add_executable(vision_CAN_reader src/vision_CAN_reader.cpp)
target_compile_options(vision_CAN_reader PRIVATE)
add_dependencies(vision_CAN_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_CAN_reader 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} vision_CAN_reader
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


############################################################################################## chassis CAN reader
add_executable(chassis_CAN_reader src/chassis_CAN_reader.cpp)
target_compile_options(chassis_CAN_reader PRIVATE)
add_dependencies(chassis_CAN_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chassis_CAN_reader 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} chassis_CAN_reader
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################# track_CAN_writer_no_grid
add_executable(track_CAN_writer_no_grid src/track_CAN_writer_no_grid.cpp)
target_compile_options(track_CAN_writer_no_grid PRIVATE)
add_dependencies(track_CAN_writer_no_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(track_CAN_writer_no_grid 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} track_CAN_writer_no_grid
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################## spat_CAN_writer
add_executable(spat_CAN_writer src/spat_CAN_writer.cpp)
target_compile_options(spat_CAN_writer PRIVATE)
add_dependencies(spat_CAN_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spat_CAN_writer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} spat_CAN_writer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################## local_CAN_writer
add_executable(local_CAN_writer src/local_CAN_writer.cpp)
target_compile_options(local_CAN_writer PRIVATE)
add_dependencies(local_CAN_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_CAN_writer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} local_CAN_writer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################ cpt7_CAN_writer
add_executable(cpt7_CAN_writer src/cpt7_CAN_writer.cpp)
target_compile_options(cpt7_CAN_writer PRIVATE)
add_dependencies(cpt7_CAN_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpt7_CAN_writer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} cpt7_CAN_writer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################## IONIQ_CAN_reader
add_executable(IONIQ_CAN_reader src/IONIQ_CAN_reader.cpp)
target_compile_options(IONIQ_CAN_reader PRIVATE)
add_dependencies(IONIQ_CAN_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IONIQ_CAN_reader 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} IONIQ_CAN_reader
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

############################################################################################## diagnostic_CAN_writer_to_ctrl
add_executable(diagnostic_CAN_writer src/diagnostic_CAN_writer.cpp)
target_compile_options(diagnostic_CAN_writer PRIVATE)
add_dependencies(diagnostic_CAN_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diagnostic_CAN_writer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} diagnostic_CAN_writer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############################################################################################## ped_detector_can_writer
add_executable(ped_detector_can_writer src/katech_ped_detector_can_writer.cpp)
target_compile_options(ped_detector_can_writer PRIVATE)
add_dependencies(ped_detector_can_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ped_detector_can_writer 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  # ${catkin_LIBRARY_DIRS}

  ##### Linking External Library
  ${CANLIB_LIBRARY}
  ${KVADBLIB_LIBRARY}
  ##############################
)
install(TARGETS ${PROJECT_NAME} ped_detector_can_writer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

