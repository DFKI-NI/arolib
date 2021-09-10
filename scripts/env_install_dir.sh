#!/bin/bash

# This script appends (in front) paths to the system env variables. The install path to be added is given as an argument. If no argument is given, the scripts directory is taken as the install path. 
# It assumes that the install path contains the following folders:
# - headers: include
# - binaries: bin, sbin
# - libraries: lib
# WARNING! : the given install path (argument 1) must be a valid path, otherwise the system variables will be corrupted! Do not add a final '/' to the path.

if [ -z "$1" ]
then
    INSTALL_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
else
    INSTALL_PATH=$1
fi

export PATH=$INSTALL_PATH/bin:$INSTALL_PATH/sbin:$PATH
export CMAKE_PREFIX_PATH=$INSTALL_PATH:$CMAKE_PREFIX_PATH
export CPATH=$INSTALL_PATH/include:$CPATH
export LD_LIBRARY_PATH=$INSTALL_PATH/lib:$LD_LIBRARY_PATH
export LIBRARY_PATH=$INSTALL_PATH/lib:$LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PATH/lib/pkgconfig:$PKG_CONFIG_PATH
