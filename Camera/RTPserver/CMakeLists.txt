cmake_minimum_required(VERSION 3.16)
project(video_process LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV configuration
set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.10.0_8")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# GStreamer configuration
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})

# Add executable
add_executable(video_process main.cpp)

# Link libraries
target_link_libraries(video_process 
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
)

# Add GStreamer library directory to runtime path
set_target_properties(video_process PROPERTIES
    INSTALL_RPATH "${GSTREAMER_LIBRARY_DIRS}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

include(GNUInstallDirs)
install(TARGETS video_process
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)