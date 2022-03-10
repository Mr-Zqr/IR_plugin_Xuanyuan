cmake_minimum_required(VERSION 3.1)

project(MyPlugin LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/MyPlugin.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/MyPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/MyPlugin.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC
  src/MyPlugin.cpp
  src/src/DTrackData.cpp
  src/src/DTrackNet.cpp
  src/src/DTrackParse.cpp
  src/src/DTrackParser.cpp
  src/src/DTrackSDK.cpp
  )

set(plugin_HDR
  src/MyPlugin.h
  src/include/DTrackDataTypes.hpp
  src/include/DTrackNet.hpp
  src/include/DTrackParse.hpp
  src/include/DTrackParser.hpp
  src/include/DTrackSDK.hpp
)

# The add_plugin macro is provided by mc_rtc
add_plugin(MyPlugin "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

target_include_directories(MyPlugin
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include/eigen-3.4.0
)
