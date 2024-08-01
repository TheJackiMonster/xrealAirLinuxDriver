cmake_minimum_required(VERSION 3.16)
project(xrealAirDebugMCU C)

set(CMAKE_C_STANDARD 17)

add_executable(
	xrealAirDebugMCU
		src/debug.c
)

target_include_directories(xrealAirDebugMCU
		BEFORE PUBLIC ${XREAL_AIR_INCLUDE_DIR}
)

target_link_libraries(xrealAirDebugMCU
		${XREAL_AIR_LIBRARY}
)
