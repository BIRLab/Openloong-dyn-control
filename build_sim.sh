#!/bin/bash

set -e

# prepare directories
rm -rf build
mkdir build

# build project
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=on
cmake --build . --target bip4_walk_wbc -- -j$(nproc)
