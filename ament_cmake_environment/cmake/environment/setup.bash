# copied from ament_cmake_environment/cmake/environment/setup.bash

AMENT_SHELL=bash

# source setup.sh from same directory as this file
AMENT_CURRENT_PREFIX=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
. "$AMENT_CURRENT_PREFIX/setup.sh"
