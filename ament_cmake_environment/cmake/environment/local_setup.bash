# copied from ament_cmake_environment/cmake/environment/local_setup.bash

AMENT_SHELL=bash

# source local_setup.sh from same directory as this file
AMENT_CURRENT_PREFIX=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
. "$AMENT_CURRENT_PREFIX/local_setup.sh"
