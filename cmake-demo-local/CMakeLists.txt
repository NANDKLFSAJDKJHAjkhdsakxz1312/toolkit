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
set(ZDL_SOFTWARE_PATH "${LOCAL_LIBRARY_BASE}/zdl-wanghuo-software")
list(APPEND CMAKE_PREFIX_PATH 
  ${QT5_PATH}
  ${ZDL_SOFTWARE_PATH}
)

find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

find_package(ZdlController REQUIRED)

# qt_add_resources(RESOURCES ../resources.qrc)

# add_executable -> target_include_directories -> target_link_libraries
add_executable(demo
    ../demo/mainwindow.ui
    ../demo/mainwindow.h
    ../demo/mainwindow.cpp
    ../demo/main.cpp
    ../src/motion_data_loader.cpp
    # ${RESOURCES}
)

target_include_directories(demo
  PRIVATE
    ../lib/include
    ../include
    ../src
)

target_link_libraries(demo
  PRIVATE
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui

    ZdlController::zdl_controller
    "${ZDL_SOFTWARE_PATH}/lib/libzdl_ecmaster.so"

    pthread
)