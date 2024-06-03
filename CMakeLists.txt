cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(picoExtRef)

pico_sdk_init()

add_executable(firmware)


target_sources(firmware PRIVATE main.c)

pico_generate_pio_header(firmware ${CMAKE_CURRENT_LIST_DIR}/26MhzOut.pio)

target_link_libraries(firmware
    pico_stdlib
    hardware_clocks
    hardware_pio
    hardware_pll
    hardware_xosc
)

pico_enable_stdio_usb(firmware 0)
pico_enable_stdio_uart(firmware 1)

pico_add_extra_outputs(firmware)

