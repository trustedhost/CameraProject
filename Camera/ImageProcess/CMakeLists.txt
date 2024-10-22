cmake_minimum_required(VERSION 3.16)

# Set app name
set(APP_NAME FaceMeshCpp)

# Set project
project(${APP_NAME})

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set 3rd party path
set(TFLite_PATH "{your library path /tflite-dist} ")
set(TFLite_INCLUDE_DIRS "${TFLite_PATH}/include")
set(TFLite_LIBS "${TFLite_PATH}/libs/mac/libtensorflowlite.dylib")

# Find required packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(CURL REQUIRED)

# Find GStreamer and its components
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

# Make executable app
add_executable(${APP_NAME})

# Add source file
add_subdirectory(src)

# Add include path
target_include_directories(${APP_NAME} 
    PRIVATE ${OpenCV_INCLUDE_DIRS} 
    PRIVATE ${TFLite_INCLUDE_DIRS}
    PRIVATE ${GST_INCLUDE_DIRS}
    PRIVATE ${GST_APP_INCLUDE_DIRS}
    PRIVATE ${CURL_INCLUDE_DIRS}
)

# Link libraries to app
target_link_libraries(${APP_NAME} 
    PRIVATE ${OpenCV_LIBS} 
    PRIVATE ${TFLite_LIBS}
    PRIVATE ${CURL_LIBRARIES}
    PRIVATE PkgConfig::GST
    PRIVATE PkgConfig::GST_APP
)

# Check if OpenMP is available
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(${APP_NAME} PRIVATE OpenMP::OpenMP_CXX)
else()
    message(WARNING "OpenMP not found. Multi-threading support may be limited.")
endif()

# Add compiler options
target_compile_options(${APP_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:AppleClang>:-Xpreprocessor -fopenmp>
)

# If using AppleClang, we need to link libomp explicitly
if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
    find_library(OpenMP_LIBRARY
        NAMES omp
        HINTS /opt/homebrew/opt/libomp/lib
    )
    if(OpenMP_LIBRARY)
        target_link_libraries(${APP_NAME} PRIVATE ${OpenMP_LIBRARY})
    else()
        message(WARNING "libomp not found. Please install it with Homebrew: brew install libomp")
    endif()
endif()