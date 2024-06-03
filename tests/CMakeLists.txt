# Host-only: portable UBX frame receiver (see ../src/ubx_rx.c). No Pico SDK.
#   cmake -S tests -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.12)
project(ubx_rx_host_test C)

add_executable(ubx_rx_test
	"${CMAKE_CURRENT_SOURCE_DIR}/ubx_rx_test.c"
	"${CMAKE_CURRENT_SOURCE_DIR}/../src/ubx_rx.c"
)
target_include_directories(ubx_rx_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_options(ubx_rx_test PRIVATE -Wall -Wextra -std=c11 -O2)

enable_testing()
add_test(NAME ubx_rx COMMAND ubx_rx_test)
