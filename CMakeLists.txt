cmake_minimum_required(VERSION 3.16.3)
project(GreenLightDetector)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

# 查找spdlog
find_package(spdlog REQUIRED)

# 查找fmt
find_package(fmt REQUIRED)

# 查找线程库
find_package(Threads REQUIRED)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 如果 find_package 成功，它会设置 EIGEN3_INCLUDE_DIR 变量
# 将这个目录添加到头文件搜索路径中
include_directories(${EIGEN3_INCLUDE_DIR})

# 包含目录 - 确保 tools 目录被包含
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/detector)
include_directories(${CMAKE_SOURCE_DIR}/tools)  # 确保这行存在
include_directories(${CMAKE_SOURCE_DIR}/io)
include_directories(${CMAKE_SOURCE_DIR}/io/hikrobot/include)
include_directories(${CMAKE_SOURCE_DIR}/io/serial/include)


# 或者使用 target_include_directories (更推荐的方式)
# ...

# 库目录
link_directories(io/hikrobot/lib/amd64)

# 添加可执行文件
add_executable(main
    main.cpp
    detector/GreenLightDetector.cpp
    io/gimbal/gimbal.cpp
    io/camera.cpp
    io/hikrobot/hikrobot.cpp
    io/serial_cboard.cpp
    tools/logger.cpp
    tools/img_tools.cpp
    tools/math_tools.cpp
    tools/crc.cpp
)

add_executable(camera_calibration
    calibration/camera_calibration.cpp
    tools/img_tools.cpp
)

# 使用target_include_directories替代include_directories
target_include_directories(main PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/detector
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/io
    ${CMAKE_SOURCE_DIR}/io/hikrobot/include
    ${CMAKE_SOURCE_DIR}/io/serial/include
)

target_include_directories(camera_calibration PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/calibration
)

# 链接库
target_link_libraries(main
    ${OpenCV_LIBS}
    yaml-cpp
    spdlog
    fmt
    usb-1.0
    MvCameraControl
    Threads::Threads
    serial
)

add_subdirectory(io/serial)

target_link_libraries(camera_calibration
    ${OpenCV_LIBS}
    yaml-cpp
    fmt
)

# 设置编译选项
target_compile_options(main PRIVATE -Wall -Wextra -g -O2)
target_compile_options(camera_calibration PRIVATE -Wall -Wextra -g -O2)