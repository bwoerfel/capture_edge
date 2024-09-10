# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040_canbus CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(capture_edge_c11b39f C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(capture_edge_c11b39f main.c capture_edge.c )

pico_set_program_name(capture_edge_c11b39f "capture_edge_c11b39f")
pico_set_program_version(capture_edge_c11b39f "0.1")

pico_generate_pio_header(capture_edge_c11b39f ${CMAKE_CURRENT_LIST_DIR}/capture_edge.pio)
pico_generate_pio_header(capture_edge_c11b39f ${CMAKE_CURRENT_LIST_DIR}/ppm.pio)
pico_generate_pio_header(capture_edge_c11b39f ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(capture_edge_c11b39f 0)
pico_enable_stdio_usb(capture_edge_c11b39f 1)

# Add the standard library to the build
target_link_libraries(capture_edge_c11b39f
        pico_stdlib
        hardware_irq
        hardware_pio
        hardware_clocks)

# Add the standard include files to the build
target_include_directories(capture_edge_c11b39f PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(capture_edge_c11b39f)

