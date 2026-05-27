CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(3D-kinect-scanner C CXX)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 ")

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Build options
option(BUILD_GUI "Build the Qt5 GUI frontend" ON)
#==============================================================================
# Dependencies
#==============================================================================

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
message(STATUS ${EIGEN3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# CUDA - prefer system default, fall back to /usr/local/cuda-8.0 if present.
if(NOT DEFINED CUDA_TOOLKIT_ROOT_DIR)
    if(EXISTS "/usr/local/cuda")
        SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
    elseif(EXISTS "/usr/local/cuda-8.0")
        SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-8.0)
    endif()
endif()
# Allow modern CMake to keep using FindCUDA (deprecated by CMP0146 in 3.27+).
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()
FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -Xcompiler -g -Xcompiler -O3 --use_fast_math)
# -gencode is GPU-arch dependent; auto-detect at configure time if not provided.
# CUDA_ARCH may be a single value ("75") or a list ("61;75;86").
if(NOT DEFINED CUDA_ARCH)
    set(CUDA_ARCH "")
    find_program(NVIDIA_SMI nvidia-smi)
    if(NVIDIA_SMI)
        execute_process(COMMAND ${NVIDIA_SMI} --query-gpu=compute_cap --format=csv,noheader
            OUTPUT_VARIABLE _detected_caps OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        string(REPLACE "\n" ";" _detected_caps "${_detected_caps}")
        foreach(_cap IN LISTS _detected_caps)
            string(REPLACE "." "" _cap "${_cap}")
            string(STRIP "${_cap}" _cap)
            if(_cap)
                list(APPEND CUDA_ARCH "${_cap}")
            endif()
        endforeach()
        list(REMOVE_DUPLICATES CUDA_ARCH)
    endif()
    if(NOT CUDA_ARCH)
        # Reasonable default covering Pascal..Turing..Ampere
        set(CUDA_ARCH "61;75;86")
    endif()
    message(STATUS "CUDA architectures: ${CUDA_ARCH}")
endif()
foreach(_arch IN LISTS CUDA_ARCH)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode=arch=compute_${_arch},code=sm_${_arch})
endforeach()

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

# Kinect
find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIR})

# ncurses
FIND_PACKAGE(Curses REQUIRED)

# Qt5 (optional)
if(BUILD_GUI)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    find_package(Qt5 5.9 COMPONENTS Core Gui Widgets OpenGL REQUIRED)
    message(STATUS "Qt5 ${Qt5Core_VERSION} found - GUI will be built")
endif()

#==============================================================================
# Application
#==============================================================================

# preprocessor define: source directory
ADD_DEFINITIONS(-DTSDF_CUDA_SOURCE_DIR=${PROJECT_SOURCE_DIR})

# application
INCLUDE_DIRECTORIES(include kinect_utilities/include ICPCUDA)

# Sophus (header-only). Use system install if available, otherwise the
# bundled copy in third_party/Sophus.
if(EXISTS "/usr/local/include/sophus/se3.hpp" OR EXISTS "/usr/include/sophus/se3.hpp")
    message(STATUS "Using system Sophus")
elseif(EXISTS "${PROJECT_SOURCE_DIR}/third_party/Sophus/sophus/se3.hpp")
    message(STATUS "Using bundled Sophus from third_party/Sophus")
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/third_party/Sophus)
else()
    message(FATAL_ERROR "Sophus not found. Either install it system-wide or run:\n"
                        "  git clone --depth 1 --branch 1.22.10 https://github.com/strasdat/Sophus.git ${PROJECT_SOURCE_DIR}/third_party/Sophus")
endif()

file(GLOB srcs ICPCUDA/ICPOdometry.cpp)
file(GLOB cuda ICPCUDA/Cuda/*.cu)
file(GLOB containers ICPCUDA/Cuda/containers/*.cpp)

if(BUILD_GUI)
    add_subdirectory(gui)
    add_definitions(-DBUILD_GUI)
endif()

CUDA_ADD_EXECUTABLE(3D-kinect-scanner src/main.cu src/marching_cubes.cpp src/VolumeIntegration.cu src/calibration.cpp
        ${srcs} ${cuda} ${containers})

if(BUILD_GUI)
    target_include_directories(3D-kinect-scanner PRIVATE gui/include)
    TARGET_LINK_LIBRARIES(3D-kinect-scanner kinect_scanner_gui)
endif()

TARGET_LINK_LIBRARIES(3D-kinect-scanner ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} -lboost_system -lpthread)

