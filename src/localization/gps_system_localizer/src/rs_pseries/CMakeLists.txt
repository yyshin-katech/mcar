cmake_minimum_required(VERSION 3.5)
project(rs_sdk_release)

#Build Localization Module
set(COMPILE_LOCALIZATION ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++14)
add_compile_options(-W)
add_compile_options(-std=c++14)

add_definitions(-DDEBUG_LEVEL=0)
add_definitions(-DRELEASE_PROJECT_PATH="${PROJECT_SOURCE_DIR}")

if(COMPILE_LOCALIZATION)
    add_definitions(-DCOMPILE_LOCALIZATION)
endif()

set(release_sdk_config_dir "${PROJECT_SOURCE_DIR}/include")

include(cmake/setting.cmake)

include(cmake/libs.cmake)

include(rs_tool/tools.cmake)

set(CONFIG_PATH ${PROJECT_SOURCE_DIR}/config)

#========================
# demo
#========================
add_subdirectory(demo)

#========================
# receiver_demo_beta
#========================
add_subdirectory(receiver_demo_beta)

