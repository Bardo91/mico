##---------------------------------------------------------------------------------------------------------------------
##  RGBD_TOOLS
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(mico VERSION 1.0 LANGUAGES C CXX)

option(BUILD_BASE "Build core module of mico" ON)
option(BUILD_FLOW "Build flow module" ON)
option(BUILD_DNN "Build dnn module" OFF)
option(BUILD_KIDS "Build kids module" ON)

option(ENABLE_TESTING "Enable unit tests in all the modules" OFF)

if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR " FATAL: In-source builds are not allowed. You should create a separate directory for build files. ")
endif()

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

if(WIN32)
  message(FATAL_ERROR "Mico is not ready to be built in Windows.")
endif()

list(APPEND MICO_MODULES_BUILD "")

# ----------------------------------------------------------------------------
# Compile modules:
# ----------------------------------------------------------------------------
if(BUILD_BASE)
  add_subdirectory(modules/base)
  list(APPEND MICO_MODULES_BUILD "base")
else()
  message(FATAL_ERROR "Mico-base is required always, do not disable it")
endif()

if(BUILD_DNN)
  add_subdirectory(modules/dnn)
  list(APPEND MICO_MODULES_BUILD "dnn")
endif()

if(BUILD_FLOW)
  add_subdirectory(modules/flow)
  list(APPEND MICO_MODULES_BUILD "flow")
endif()

if(BUILD_KIDS)
  if(NOT BUILD_FLOW)
    message(FATAL_ERROR "mico-flow is required to build mico-kids and it is not being build.")
  endif()
  add_subdirectory(modules/kids)
  list(APPEND MICO_MODULES_BUILD "kids")
endif()

# ----------------------------------------------------------------------------
# Install:
# ----------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/templates/mico-config.cmake.in"
	"${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
	@ONLY
)

write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  VERSION ${mico_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${PROJECT_NAME}
)



# ----------------------------------------------------------------------------
# Summary:
# ----------------------------------------------------------------------------
message(STATUS "==========================================================================")
message(STATUS "==========================================================================")
message(STATUS "")
message(STATUS "General configuration for mico ${mico_VERSION} ")
message(STATUS "    Machine:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "    CMake:"            ${CMAKE_VERSION})
message(STATUS "    CMake generator:"  ${CMAKE_GENERATOR})
message(STATUS "    CMake build tool:" ${CMAKE_BUILD_TOOL})
message(STATUS "")
message(STATUS "  C/C++:")
message(STATUS "    C++ Compiler:"           ${CMAKE_CXX_COMPILER})
message(STATUS "    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
message(STATUS "    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
message(STATUS "    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
message(STATUS "    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
message(STATUS "    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
message(STATUS "    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
message(STATUS "    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})
message(STATUS "")
message(STATUS "  mico modules: ${MICO_MODULES_BUILD}")
message(STATUS "")
message(STATUS "  Install to:" "${CMAKE_INSTALL_PREFIX}")
message(STATUS "==========================================================================")
message(STATUS "")
