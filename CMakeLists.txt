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

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(detector)
include_directories(serial)
include_directories(tools)
include_directories(io)
include_directories(io/hikrobot/include)

# 库目录
link_directories(io/hikrobot/lib/amd64)

# 添加可执行文件
add_executable(main
    main.cpp
    detector/GreenLightDetector.cpp
    serial/serial.cpp
    serial/crc.cpp
    io/camera.cpp
    io/hikrobot/hikrobot.cpp
    tools/logger.cpp
)

add_executable(camera_calibration
    calibration/camera_calibration.cpp
)

# 链接库
target_link_libraries(main
    ${OpenCV_LIBS}
    yaml-cpp
    spdlog
    fmt
    usb-1.0
    MvCameraControl
)

target_link_libraries(camera_calibration
    ${OpenCV_LIBS}
    yaml-cpp
)

# 设置编译选项
target_compile_options(main PRIVATE -Wall -Wextra -g -O2)
