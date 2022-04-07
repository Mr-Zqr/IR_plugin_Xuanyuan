cmake_minimum_required(VERSION 3.1)

project(IRPlugin LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/IRPlugin.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/IRPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/IRPlugin.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(DTrackSDK_SRC
  src/DTrackSDK/src/DTrackData.cpp
  src/DTrackSDK/src/DTrackNet.cpp
  src/DTrackSDK/src/DTrackParse.cpp
  src/DTrackSDK/src/DTrackParser.cpp
  src/DTrackSDK/src/DTrackSDK.cpp
)

set(plugin_SRC
  src/IRPlugin.cpp
  ${DTrackSDK_SRC}
)

set(DTrackSDK_HDR
  src/DTrackSDK/include/DTrackDataTypes.hpp
  src/DTrackSDK/include/DTrackNet.hpp
  src/DTrackSDK/include/DTrackParse.hpp
  src/DTrackSDK/include/DTrackParser.hpp
  src/DTrackSDK/include/DTrackSDK.hpp
)

set(plugin_HDR
  src/IRPlugin.h
  ${DTrackSDK_HDR}
)

# The add_plugin macro is provided by mc_rtc
add_plugin(IRPlugin "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

target_include_directories(IRPlugin
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src/DTrackSDK/include
)
