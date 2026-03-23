cmake_minimum_required(VERSION 3.10)

project(dds_ui_cmd_server VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ZDL_SOFTWARE_DIR "/home/root/software/zdl-wanghuo-software")

# 源文件
set(SOURCES
    ../main.cpp
    ../ui_cmd_server.cpp
    ../idl/start_task.c
    ../utils/struct_init.cpp
    ../idl/stop_task.c
    ../idl/enable.c
    ../idl/disable.c
    ../idl/set_zero_position.c
    ../src/motion_data_loader.cpp
    ../idl/return_zero.c
    ../idl/jog_command.c
    ../idl/robot_state.c
    ../idl/waist_target.c
    ../idl/wheel_stop.c
    ../idl/estop.c
    ../linear_interpolator.cpp
    ../idl/csp_command.c
)

# 依赖路径
set(CMAKE_PREFIX_PATH
    "/home/root/software/cyclonedds0.10.5_install"
    "/home/root/software/spdlog1.16.0_install"
    "/home/root/software/boost_1.85.0_install"
    "/home/root/software/eigen3.4.0_install"
)

# ZdlController
set(ZdlController_DIR "${ZDL_SOFTWARE_DIR}/lib/cmake/ZdlController")

# 找 包
find_package(ZdlController REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(spdlog REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(Boost 1.85 REQUIRED COMPONENTS filesystem serialization)

# 可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# ✅ 正确的 include（关键修复）
target_include_directories(${PROJECT_NAME} PRIVATE
    ${ZDL_SOFTWARE_DIR}/lib/include
    ../include
   
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ZdlController::zdl_controller
    ${ZDL_SOFTWARE_DIR}/lib/libzdl_ecmaster.so
    Eigen3::Eigen
    CycloneDDS::ddsc
    ${Boost_LIBRARIES}
    spdlog::spdlog
)