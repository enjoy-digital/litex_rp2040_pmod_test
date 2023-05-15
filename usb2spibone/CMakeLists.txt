# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(usb2spibone C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

set(FAMILY rp2040)
set(BOARD pico_sdk)
set(TINYUSB_FAMILY_PROJECT_NAME_PREFIX "tinyusb_dev_")
add_executable(dev_lowlevel
        dev_lowlevel.c
        )

target_link_libraries(dev_lowlevel PRIVATE pico_stdlib hardware_resets hardware_irq)
pico_add_extra_outputs(dev_lowlevel)

# add url via pico_set_program_url
# usb2spibone_auto_set_url(dev_lowlevel)