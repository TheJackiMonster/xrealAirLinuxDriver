cmake_minimum_required(VERSION 3.16)
project(xrealAirLibrary C)

set(CMAKE_C_STANDARD 17)

find_package(json-c REQUIRED CONFIG)

add_subdirectory(modules/hidapi)
add_subdirectory(modules/Fusion/Fusion)

add_library(
		xrealAirLibrary
		src/crc32.c
		src/device.c
		src/device_imu.c
		src/device_mcu.c
		src/hid_ids.c
)

target_compile_options(xrealAirLibrary PRIVATE -fPIC)

target_include_directories(xrealAirLibrary
		BEFORE PUBLIC include
)

target_include_directories(xrealAirLibrary
		SYSTEM BEFORE PRIVATE 
		${CMAKE_CURRENT_SOURCE_DIR}/modules/hidapi
		${CMAKE_CURRENT_SOURCE_DIR}/modules/Fusion
)

target_link_libraries(xrealAirLibrary
		PRIVATE hidapi::hidapi json-c::json-c Fusion m
)

set(XREAL_AIR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(XREAL_AIR_LIBRARY xrealAirLibrary PARENT_SCOPE)

set(NREAL_AIR_INCLUDE_DIR ${XREAL_AIR_INCLUDE_DIR} PARENT_SCOPE)
set(NREAL_AIR_LIBRARY ${XREAL_AIR_LIBRARY} PARENT_SCOPE)
