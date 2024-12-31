#!/bin/bash

set -e

# prepare directories
rm -rf build install
mkdir build install

# build project
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=on
cmake --build . --target main -- -j$(nproc)

# install project
cmake --install . --prefix ../install
