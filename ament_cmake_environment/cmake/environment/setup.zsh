# copied from ament_cmake_environment/cmake/environment/setup.zsh

AMENT_SHELL=zsh

# source setup.sh from same directory as this file
AMENT_CURRENT_PREFIX=$(builtin cd -q "`dirname "${(%):-%N}"`" > /dev/null && pwd)
. "$AMENT_CURRENT_PREFIX/setup.sh"
