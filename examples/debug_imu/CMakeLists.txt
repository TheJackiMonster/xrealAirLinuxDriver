cmake_minimum_required(VERSION 3.16)
project(xrealAirDebugIMU C)

set(CMAKE_C_STANDARD 17)

add_executable(
	xrealAirDebugIMU
		src/debug.c
)

target_include_directories(xrealAirDebugIMU
		BEFORE PUBLIC ${XREAL_AIR_INCLUDE_DIR}
)

target_link_libraries(xrealAirDebugIMU
		${XREAL_AIR_LIBRARY}
)
