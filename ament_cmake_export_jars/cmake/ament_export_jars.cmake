# Copyright 2016 Esteve Fernandez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Export JAR files to downstream packages.
#
# :param ARGN: a list of JAR files.
#   Each element might either be an absolute path to a JAR file.
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_jars)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_jars() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_jars_register_package_hook()
    foreach(_arg ${ARGN})
      if(IS_ABSOLUTE "${_arg}")
        if(NOT EXISTS "${_arg}")
          message(WARNING
            "ament_export_jars() package '${PROJECT_NAME}' exports the "
            "jar '${_arg}' which doesn't exist")
        endif()
        list_append_unique(_AMENT_EXPORT_ABSOLUTE_JARS "${_arg}")
      else()
        set(_arg "\${${PROJECT_NAME}_DIR}/../../../${_arg}")
        list_append_unique(_AMENT_EXPORT_RELATIVE_JARS "${_arg}")
      endif()
    endforeach()
  endif()
endmacro()
