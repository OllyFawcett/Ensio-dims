cmake_minimum_required(VERSION 3.5)


include_directories("C:/Users/Olly/Downloads/opencv/build/include")
include_directories("C:/Users/Olly/vcpkg/installed/x64-windows/include")
link_directories("C:/Users/Olly/Downloads/opencv/build/x64/vc15/lib")
link_directories("C:/Users/Olly/vcpkg/installed/x64-windows/lib")
file(GLOB LIB_FILES "C:/Users/Olly/vcpkg/installed/x64-windows/lib/* .lib")


project(dims_calculator VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)



set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        camerathread.cpp
        camerathread.h
        dims_tools.hpp
        Combinear.qss

)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(dims_calculator
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET dims_calculator APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(dims_calculator SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(dims_calculator
            ${PROJECT_SOURCES}
        )
    endif()
endif()
target_link_libraries(dims_calculator PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${LIB_FILES} "opencv_world460d" "realsense2" "opencv_calib3d2" "opencv_contrib2" "opencv_core2" "opencv_features2d2" "opencv_flann2" "opencv_gpu2" "opencv_highgui2" "opencv_imgproc2" "opencv_legacy2" "opencv_ml2" "opencv_nonfree2" "opencv_objdetect2" "opencv_ocl2" "opencv_photo2" "opencv_stitching2" "opencv_ts2" "opencv_video2" "opencv_videostab2")

set_target_properties(dims_calculator PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS dims_calculator
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(dims_calculator)
endif()
