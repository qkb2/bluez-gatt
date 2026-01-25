cmake_minimum_required(VERSION 3.16)
project(iot_ble_server)

# config options
option(VERBOSE "Enable verbose logging" OFF)

set(BLE_MAC "" CACHE STRING "Target BLE device MAC address (AA:BB:CC:DD:EE:FF)")

if (BLE_MAC STREQUAL "")
    message(FATAL_ERROR "BLE_MAC must be set (e.g. -DBLE_MAC=AA:BB:CC:DD:EE:FF)")
endif()

# Configure header
configure_file(
    includes/config.h.in
    includes/config.h
)

# C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_compile_definitions(
    _GNU_SOURCE
    _DEFAULT_SOURCE
)

# Includes
include_directories(includes)
include_directories(includes/shared)

# Subdirectories
# add_subdirectory(libbluetooth)
add_subdirectory(libshared)

# Source files
file(GLOB_RECURSE SRC_FILES src/*.c)

# Target
add_executable(iot_ble_server ${SRC_FILES})

target_include_directories(iot_ble_server PRIVATE
    ${CMAKE_SOURCE_DIR}/includes
    ${CMAKE_BINARY_DIR}/includes
    /usr/include/bluetooth
)

# Link libraries
target_link_libraries(iot_ble_server
    pthread
    bluetooth
    shared
)

# Respect Buildroot sysroot
set_target_properties(iot_ble_server PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Optional install target
install(TARGETS iot_ble_server
    RUNTIME DESTINATION bin
)