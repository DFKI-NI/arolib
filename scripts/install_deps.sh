#!/bin/sh

# This script installs all dependencies of this project. It is also called by
# odil_ws/scripts/odil_install_deps, so don't rename!

SUDO=''
if [ $(id -u) -ne 0 ]; then
    SUDO='sudo'
fi

$SUDO apt update

$SUDO apt-get install -qq -y \
    cmake \
    g++ \
    libboost-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-test-dev \
    libgdal-dev \
    libshp-dev \
    libpng++-dev \
    libcrypto++-dev \
    libhdf5-dev
