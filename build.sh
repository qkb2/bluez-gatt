#!/bin/bash

set -e

BLE_MAC="$1"
VERBOSE="${2:-OFF}"

BUILD_DIR=build
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DBLE_MAC="$BLE_MAC" -DVERBOSE="$VERBOSE"

cmake --build . -- -j$(nproc)

echo "Build finished. Executable is at $BUILD_DIR/ble_http_server"