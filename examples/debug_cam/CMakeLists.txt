cmake_minimum_required(VERSION 3.16)
project(xrealAirDebugCamera CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    xrealAirDebugCamera
    src/camera.cpp
)

target_include_directories(xrealAirDebugCamera
    BEFORE PUBLIC ${XREAL_AIR_INCLUDE_DIR}
)

target_link_libraries(xrealAirDebugCamera
    ${XREAL_AIR_LIBRARY} ${OpenCV_LIBS}
)
