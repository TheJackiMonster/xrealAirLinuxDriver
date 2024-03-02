cmake_minimum_required(VERSION 3.16)
project(xrealAirLinuxDriver C)

set(CMAKE_C_STANDARD 17)

add_subdirectory(interface_lib)
add_subdirectory(examples)

add_executable(xrealAirLinuxDriver
		src/driver.c
)

target_include_directories(xrealAirLinuxDriver
		BEFORE PUBLIC ${XREAL_AIR_INCLUDE_DIR}
)

target_link_libraries(xrealAirLinuxDriver
		${XREAL_AIR_LIBRARY}
)