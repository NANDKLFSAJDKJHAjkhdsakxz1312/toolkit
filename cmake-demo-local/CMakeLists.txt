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

# 手动指定依赖库路径
set(EIGEN3_INCLUDE_DIR "${LOCAL_LIBRARY_BASE}/eigen3.4.0_install/include/eigen3")
set(SPDLOG_INCLUDE_DIR "${LOCAL_LIBRARY_BASE}/spdlog1.16.0_install/include")
set(YAML_CPP_INCLUDE_DIR "${LOCAL_LIBRARY_BASE}/yaml-cpp0.8.0_install/include")
set(YAML_CPP_LIBRARIES "${LOCAL_LIBRARY_BASE}/yaml-cpp0.8.0_install/lib64/libyaml-cpp.so")
set(BOOST_LIBRARY_DIR "${LOCAL_LIBRARY_BASE}/boost_1.85.0_install/lib")

# 查找 Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

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
    ${EIGEN3_INCLUDE_DIR}
    ${SPDLOG_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(demo
  PRIVATE
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    ${YAML_CPP_LIBRARIES}
    ${BOOST_LIBRARY_DIR}/libboost_filesystem.so
    ${BOOST_LIBRARY_DIR}/libboost_serialization.so

    "${CMAKE_CURRENT_SOURCE_DIR}/../lib/libzdl_controller_static.so"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ecmaster/libzdl_ecmaster.so"

    pthread
)