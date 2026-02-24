cmake_minimum_required(VERSION 3.16)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.30")
    cmake_policy(SET CMP0167 OLD)
endif()

project(ws_ZDL_EC_MoveCapture LANGUAGES CXX)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(LOCAL_LIBRARY_BASE "/home/root/software")
set(QT5_PATH "${LOCAL_LIBRARY_BASE}/qt5.15.18_install")

list(APPEND CMAKE_PREFIX_PATH ${QT5_PATH})

find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
qt_add_resources(RESOURCES resources.qrc)

add_executable(demo
    demo/mainwindow.ui
    demo/mainwindow.h
    demo/mainwindow.cpp
    demo/main.cpp
    ${RESOURCES}
)

target_include_directories(demo
  PRIVATE
    ./lib/include
    ./include
)

set(ZDL_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/libzdl_controller.so")
target_link_libraries(demo
  PRIVATE 
    "${ZDL_LIB_PATH}"
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
)

set(BOOST_LIB_DIR "/home/root/software/boost_1.85.0_install/lib")

target_link_options(demo PRIVATE 
    "-Wl,-rpath-link,${BOOST_LIB_DIR}"
)
