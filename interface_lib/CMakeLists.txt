cmake_minimum_required(VERSION 3.16)
project(nrealAirLibrary C)

set(CMAKE_C_STANDARD 17)

find_package(json-c REQUIRED CONFIG)

add_subdirectory(modules/hidapi)
add_subdirectory(modules/Fusion/Fusion)

add_library(
		nrealAirLibrary
		src/crc32.c
		src/device3.c
		src/device4.c
)

target_compile_options(nrealAirLibrary PRIVATE -fPIC)

target_include_directories(nrealAirLibrary
		BEFORE PUBLIC include
)

target_include_directories(nrealAirLibrary
		SYSTEM BEFORE PRIVATE 
		${CMAKE_CURRENT_SOURCE_DIR}/modules/hidapi
		${CMAKE_CURRENT_SOURCE_DIR}/modules/Fusion
)

target_link_libraries(nrealAirLibrary
		PRIVATE hidapi::hidapi json-c::json-c Fusion m
)

set(NREAL_AIR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(NREAL_AIR_LIBRARY nrealAirLibrary PARENT_SCOPE)
