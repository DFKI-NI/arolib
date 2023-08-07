#!/bin/bash

# Probably no longer required, the main Makefile doesn't source this file anymore

if [ "$AROLIB_AROLIB_ENV_SET" != "TRUE" ]; then
	echo "Setting AroLib - arolib env variables.."

	SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

	export AROLIB_AROLIB_ROOT=${AROLIB_AROLIB_ROOT:-"$SCRIPT_DIR/.."}
	export AROLIB_AROLIB_INSTALL_PATH=${AROLIB_AROLIB_INSTALL_PATH:-"$HOME/arolib/install/arolib"}

	source $SCRIPT_DIR/env_install_dir.sh $AROLIB_AROLIB_INSTALL_PATH

	export AROLIB_AROLIB_ENV_SET="TRUE"
fi
