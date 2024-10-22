cmake_minimum_required(VERSION 3.10)
project(rtsp_server)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED 
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-rtsp-server-1.0
)

# Add executable
add_executable(rtsp_server rtsp_server.cpp)

# Include directories
target_include_directories(rtsp_server PRIVATE 
    ${GSTREAMER_INCLUDE_DIRS}
)

# Link libraries
target_link_directories(rtsp_server PRIVATE 
    ${GSTREAMER_LIBRARY_DIRS}
)

target_link_libraries(rtsp_server PRIVATE 
    ${GSTREAMER_LIBRARIES}
    pthread
)

# Add compile options
target_compile_options(rtsp_server PRIVATE 
    ${GSTREAMER_CFLAGS_OTHER}
)