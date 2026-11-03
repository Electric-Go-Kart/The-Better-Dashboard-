# Standalone build of the desktop lidar test driver (needs only Qt Core + SerialPort, not the full dash).
#   cmake -S tools -B build/lidar_harness -G Ninja -DCMAKE_PREFIX_PATH=<Qt>/gcc_64
#   cmake --build build/lidar_harness
cmake_minimum_required(VERSION 3.16)
project(LidarHarness LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Qt6 REQUIRED COMPONENTS Core SerialPort)

set(BACKEND ${CMAKE_CURRENT_SOURCE_DIR}/../Backend)
add_executable(lidarHarnessCli
    lidar_harness_cli.cpp
    ${BACKEND}/stl27lreader.cpp
    ${BACKEND}/lidarpipeline.cpp
)
target_include_directories(lidarHarnessCli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_options(lidarHarnessCli PRIVATE -Wall -Wextra)
target_link_libraries(lidarHarnessCli PRIVATE Qt6::Core Qt6::SerialPort)
set_target_properties(lidarHarnessCli PROPERTIES AUTOMOC ON)
