#!/bin/bash

set -e

add-apt-repository -y ppa:lely/ppa
apt update
apt install -y \
    git \
    cmake \
    gcc-11 \
    g++-11 \
    libglu1-mesa-dev \
    freeglut3-dev \
    python3-yaml \
    liblely-coapp-dev \
    liblely-co-tools \
    python3-dcf-tools
