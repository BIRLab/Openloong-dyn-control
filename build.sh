#!/bin/bash

set -e

# prepare directories
rm -rf build install
mkdir build install

# build project
cd build
cmake ..
cmake --build . --target main -- -j$(nproc)

# install project
cmake --install . --prefix ../install
