#!/bin/bash

set -e

BLE_MAC="$1"
VERBOSE="${2:-OFF}"
USE_TOOLCHAIN="${3:-OFF}"

if [[ "$USE_TOOLCHAIN" == "musl" ]]; then
  BUILD_DIR=build-musl
  TOOLCHAIN_ARG="-DCMAKE_TOOLCHAIN_FILE=toolchains/musl-gcc.cmake"
  echo "Building with musl"
else
  BUILD_DIR=build
  TOOLCHAIN_ARG=""
  echo "Building with system toolchain"
fi

BUILD_DIR=build
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. \
  -DBLE_MAC="$BLE_MAC" \
  -DVERBOSE="$VERBOSE" \
  $TOOLCHAIN_ARG

cmake --build . -- -j$(nproc)

echo "Build finished. Executable is at $BUILD_DIR/ble_http_server"