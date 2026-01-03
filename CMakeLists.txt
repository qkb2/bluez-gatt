cmake_minimum_required(VERSION 3.16)
project(ble_http_server)

# config options
option(VERBOSE "Enable verbose logging" OFF)

set(BLE_MAC "" CACHE STRING "Target BLE device MAC address (AA:BB:CC:DD:EE:FF)")

if (BLE_MAC STREQUAL "")
    message(FATAL_ERROR "BLE_MAC must be set (e.g. -DBLE_MAC=AA:BB:CC:DD:EE:FF)")
endif()

# Configure header
configure_file(
    src/config.h.in
    src/config.h
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
include_directories(includes/src/shared)
include_directories(includes/lib)

# Subdirectories
add_subdirectory(libbluetooth)
add_subdirectory(libshared)

# Source files
file(GLOB_RECURSE SRC_FILES src/*.c)

# Target
add_executable(ble_http_server ${SRC_FILES})

target_include_directories(ble_http_server PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/src
)

# Link libraries
target_link_libraries(ble_http_server
    pthread
    bluetooth
    shared
)

# Respect Buildroot sysroot
set_target_properties(ble_http_server PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Optional install target
install(TARGETS ble_http_server
    RUNTIME DESTINATION bin
)