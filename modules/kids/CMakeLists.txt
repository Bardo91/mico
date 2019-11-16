##---------------------------------------------------------------------------------------------------------------------
##  MICO-KIDS
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
set(COMPONENT_NAME ${PROJECT_NAME}-kids)

##################################################
###### Create project and configure ALIAS ########
##################################################
set(CMAKE_AUTOMOC ON)

file(GLOB_RECURSE MICO_KIDS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_KIDS_HEADER_FILES "include/*.h" "include/*.inl" "include/*.hpp")
 

add_executable(${COMPONENT_NAME} ${MICO_KIDS_HEADER_FILES} ${MICO_KIDS_SOURCE_FILES})
target_link_libraries(${COMPONENT_NAME} LINK_PRIVATE mico::mico-base mico::mico-flow)
target_compile_features(${COMPONENT_NAME} PUBLIC cxx_std_17)
target_include_directories(${COMPONENT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${COMPONENT_NAME} PUBLIC pthread)

##########################################
######		Configure project		######
##########################################

#########################################
###### Loading 3rd party libraries.######
#########################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

target_compile_options(${COMPONENT_NAME} PUBLIC "-Wall")

target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC mico::mico-base mico::mico-flow)

find_package(NodeEditor REQUIRED)

target_include_directories(${COMPONENT_NAME} PUBLIC ${NodeEditor_INCLUDE_DIRS})
target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${NodeEditor_LIBRARIES})

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 


install(TARGETS ${COMPONENT_NAME} EXPORT ${COMPONENT_NAME}-targets
		COMPONENT ${COMPONENT_NAME}
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include
	  )


install(EXPORT ${COMPONENT_NAME}-targets
	FILE "${COMPONENT_NAME}-targets.cmake"
	NAMESPACE ${PROJECT_NAME}::
	DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT ${COMPONENT_NAME}
)

configure_file("${COMPONENT_NAME}-config.cmake.in"
	"${CMAKE_BINARY_DIR}/${COMPONENT_NAME}-config.cmake"
	@ONLY
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/${COMPONENT_NAME}-config-version.cmake"
  VERSION ${version}
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${CMAKE_BINARY_DIR}/${COMPONENT_NAME}-config.cmake"
    "${CMAKE_BINARY_DIR}/${COMPONENT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  COMPONENT ${COMPONENT_NAME}
)

# # DESTINATION will be automatically prefixed by ${CMAKE_INSTALL_PREFIX}
# install(
#   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
#   COMPONENT ${COMPONENT_NAME}
#   DESTINATION ${CMAKE_INSTALL_PREFIX}
# )

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
