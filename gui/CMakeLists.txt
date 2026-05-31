# gui/CMakeLists.txt — Qt5 GUI frontend for kinect_scanner
#
# Static library `kinect_scanner_gui` exposing `int runGui(int, char**)`.
# Linked into the main 3D-kinect-scanner executable when BUILD_GUI=ON.

set(GUI_SOURCES
    src/gui_entry.cpp
    src/main_window.cpp
    src/scanner_worker.cpp
    src/actuator_tcp_server.cpp
    src/simulated_frame_source.cpp
    src/tcp_control_server.cpp
    src/parameter_panel.cpp
    src/image_panel.cpp
    src/viewer3d.cpp
    src/preset_io.cpp
)

set(GUI_HEADERS
    include/gui_entry.hpp
    src/main_window.hpp
    src/scanner_worker.hpp
    src/actuator_tcp_server.hpp
    src/simulated_frame_source.hpp
    src/tcp_control_server.hpp
    src/parameter_panel.hpp
    src/image_panel.hpp
    src/viewer3d.hpp
    src/preset_io.hpp
)

add_library(kinect_scanner_gui STATIC ${GUI_SOURCES} ${GUI_HEADERS})

target_include_directories(kinect_scanner_gui
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/kinect_utilities/include
            ${CMAKE_SOURCE_DIR}/ICPCUDA
            ${CMAKE_SOURCE_DIR}/third_party/Sophus
            ${OpenCV_INCLUDE_DIRS}
            ${freenect2_INCLUDE_DIR}
            ${CUDA_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(kinect_scanner_gui
    PUBLIC  Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL Qt5::Network
    PRIVATE ${OpenCV_LIBS}
)

# Position-independent code so the static lib can link into the main exe.
set_target_properties(kinect_scanner_gui PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)
