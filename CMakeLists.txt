cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(picoExtRef)

set(FW_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(FW_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(FW_GIT_DESCRIBE "unknown")
find_program(GIT_EXECUTABLE git)
if(GIT_EXECUTABLE)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} describe --always --dirty
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
		OUTPUT_VARIABLE FW_GIT_DESCRIBE
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
endif()

set(FW_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${FW_GEN_DIR}")
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/firmware_version.h.in"
	"${FW_GEN_DIR}/firmware_version.h"
	@ONLY
)

pico_sdk_init()

add_executable(firmware)

target_include_directories(firmware PRIVATE "${FW_INC_DIR}" "${FW_GEN_DIR}")

target_sources(firmware PRIVATE
	"${FW_SRC_DIR}/main.c"
	"${FW_SRC_DIR}/gpsdo_control.c"
	"${FW_SRC_DIR}/gpsdo_tune.c"
	"${FW_SRC_DIR}/ubx.c"
	"${FW_SRC_DIR}/ubx_rx.c"
)

target_link_libraries(firmware
    pico_stdlib
    hardware_clocks
    hardware_pwm
    hardware_i2c
    hardware_uart
)

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)

# USB identity for CDC (stdio): pid.codes test VID 0x1209 (not Raspberry Pi 0x2e8a) + project PID — unambiguous vs Picoprobe (2e8a:000c).
# Define after pico_enable_stdio_usb (matches common Pico SDK examples). If lsusb still shows 2e8a:000a, wipe build/ and rebuild; ./pico.py build checks for the Orangebox string in the ELF.
# Register a free PID at https://pid.codes if you ship hardware; ./pico.py flash defaults match below.
# 1 = on boot, drop/re-assert USB pull-up so the host re-enumerates (no cable unplug). Set to 0 to skip.
target_compile_definitions(firmware PRIVATE
    USBD_VID=0x1209
    USBD_PID=0x0C01
    "USBD_MANUFACTURER=\"Orangebox\""
    "USBD_PRODUCT=\"picoExtRef-GPSDO\""
    PICO_USB_BOOT_REENUM=1
)

pico_add_extra_outputs(firmware)

