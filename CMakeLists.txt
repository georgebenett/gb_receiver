# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# The app descriptor carries FW_VERSION, not git describe: remotes read it from
# the bundled gb_receiver.bin to decide whether to push it over BLE.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/main/version.h" _fw_ver REGEX "#define FW_VERSION")
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PROJECT_VER "${_fw_ver}")
# Re-configure on a version bump; otherwise incremental builds keep the old one.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/main/version.h")
project(gb_receiver)
