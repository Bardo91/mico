##---------------------------------------------------------------------------------------------------------------------
##  MICO-BASE
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
set(COMPONENT_NAME ${PROJECT_NAME}-base)

##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE RGDBTOOLS_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE RGDBTOOLS_HEADER_FILES "include/*.h" "include/*.inl")
 
add_library(${COMPONENT_NAME} SHARED ${RGDBTOOLS_HEADER_FILES} ${RGDBTOOLS_SOURCE_FILES})
target_compile_features(${COMPONENT_NAME} PUBLIC cxx_std_11)
target_include_directories(${COMPONENT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${COMPONENT_NAME} PUBLIC pthread)

add_library(${PROJECT_NAME}::${COMPONENT_NAME} ALIAS ${COMPONENT_NAME})

##########################################
######		Configure project		######
##########################################

# SLAM modules
option(USE_MODULE_SLAM		"Compile the slam module"					ON)
option(USE_G2O				"Using g2o for the optimization module" 	ON)
option(USE_DBOW2			"Using DBoW2 in the slam algorithms"	 	ON)
option(MICO_USE_ROS			"Compile ROS dependent modules"	 			OFF)

option(USE_MAVLINK 			"Compile MAVLINK interface classes"			ON)

# Others
option(BUILD_TOOLS			"Compile mico-base tools" 					ON)
option(BUILD_EXAMPLES		"Compile mico-base examples"				ON)
option(BUILD_TESTS		"Compile mico-base tests"				ON)

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

#########################################
## Load librealsense
find_package(realsense2 HINTS "/usr/local/lib/cmake/realsense2")
if(${realsense2_FOUND})
	MESSAGE(STATUS "Found librealsense v2, compiling RealSense devices wrapper")
	target_compile_definitions(${COMPONENT_NAME} PUBLIC "ENABLE_LIBREALSENSE_V2")
	target_include_directories(${COMPONENT_NAME} PUBLIC ${realsense2_INCLUDE_DIRS})
    target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${realsense2_LIBRARY})
else()
	find_package(librealsense)
	if(${LIBREALSENSE_FOUND})
		MESSAGE(STATUS "Found librealsense v1, compiling RealSense devices wrapper")
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "ENABLE_LIBREALSENSE_V1")
		target_include_directories(${COMPONENT_NAME} PUBLIC ${LIBREALSENSE_INCLUDE_DIRS})
		target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${LIBREALSENSE_LIBRARIES})
	endif()
endif()


#########################################
## Load freenect
find_package(libfreenect)
if(${LIBFREENECT_FOUND})
	MESSAGE(STATUS "Found libfreenect, compiling kinect wrapper")
	target_compile_definitions(${COMPONENT_NAME} PUBLIC "ENABLE_LIBFREENECT")
	target_include_directories(${COMPONENT_NAME} PUBLIC ${LIBFREENECT_INCLUDE_DIRS})
    target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${LIBFREENECT_LIBRARY})
endif()

if(${USE_MODULE_SLAM})
	target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${dlib_LIBRARIES})
	if(${USE_DBOW2})
		find_package(DBoW2)
		find_package(DLoopDetector)
		find_package(DLib)

		if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1") AND (${DLoopDetector_FOUND} EQUAL "1"))
			target_compile_definitions(${COMPONENT_NAME} PUBLIC "USE_DBOW2")	
			target_include_directories(${COMPONENT_NAME} PUBLIC ${DBoW2_INCLUDE_DIR})
			target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${DBoW2_LIBRARY})
			target_include_directories(${COMPONENT_NAME} PUBLIC ${DLoopDetector_INCLUDE_DIR})
			#target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${DLoopDetector_LIBRARY})
			target_include_directories(${COMPONENT_NAME} PUBLIC /usr/local/include/DUtils)	## WHY is it not working with variable?...
			target_include_directories(${COMPONENT_NAME} PUBLIC /usr/local/include/DVision)
			target_include_directories(${COMPONENT_NAME} PUBLIC /usr/local/include/DUtilsCV) # ${DLib_INCLUDE_DIR})
			target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${DLib_LIBRARY})
		else()
			message(Warning "DBOoW2 not found, disabling SLAM module")
			SET(USE_DBOW2 OFF)
			SET(USE_G2O OFF)
			SET(USE_MODULE_SLAM OFF)
		endif()
	endif(${USE_DBOW2})

	# g2o dependencies
	if(${USE_G2O})
		set(G2O_ROOT /usr/local)
		FIND_PACKAGE(Eigen3 REQUIRED)
		FIND_PACKAGE(CSparse REQUIRED)
		FIND_PACKAGE(Cholmod REQUIRED)
		FIND_PACKAGE(OpenGL REQUIRED)
		FIND_PACKAGE(g2o HINTS "/usr/local/lib/cmake/g2o")
		if(${g2o_FOUND})
			target_compile_definitions(${COMPONENT_NAME} PUBLIC "USE_G2O")	
			target_include_directories(${COMPONENT_NAME}  PUBLIC ${CHOLMOD_INCLUDE_DIR})
			
			target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC lapack)
			target_link_libraries(${COMPONENT_NAME}  LINK_PUBLIC g2o::core g2o::types_sba g2o::solver_cholmod)
		else()
			message(Warning "G2O not found, disabling SLAM module")
			set(USE_G2O OFF)
			SET(USE_MODULE_SLAM OFF)
		endif()
	endif(${USE_G2O})
endif(${USE_MODULE_SLAM})

#########################################
## Load opencv
find_package(OpenCV REQUIRED)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

target_include_directories(${COMPONENT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

if(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "4")
	target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_OPENCV_4")
elseif(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "3")
	target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_OPENCV_3")
else()
	MESSAGE(FATAL_ERROR "Not possible to compile with OpenCV 2")
endif()

#########################################
## Load PCL
find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	target_include_directories(${COMPONENT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
	
	target_compile_definitions(${COMPONENT_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")
	# target_compile_options(${COMPONENT_NAME} PUBLIC "-march=native -msse4.2 -mfpmath=sse -L/usr/lib/x86_64-linux-gnu")

	if(${PCL_VERSION} VERSION_GREATER 1.8)
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_PCL_1_8")
	else()
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_PCL_1_7")
	endif()

else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

#########################################
## Load Artec SDK 
if(WIN32) # Look for a better way of doing this.
	if(${CMAKE_EXE_LINKER_FLAGS} MATCHES  "[/]([a-z])+[:]+x64")
		message(STATUS "Using ARTEC SDK with arch x64")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin-x64/*.lib")
	elseif(${CMAKE_EXE_LINKER_FLAGS} MATCHES  "[/]([a-z])+[:]+x86")
	message(STATUS "Using ARTEC SDK with arch x86")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin/*.lib")
	else()
		message(WARNING "Can't detect architecture configuration, assuming x86")
		file(GLOB_RECURSE ARTEC_LIBS "C:/Program Files/Artec/Artec 3D Scanning SDK/bin/*.lib")
	endif()
	
	set(ARTEC_INCLUDE_DIR "C:/Program Files/Artec/Artec 3D Scanning SDK/include")
	if(ARTEC_LIBS STREQUAL "")
		MESSAGE(WARNING "Couldn't found ARTEC SDK")
	else()
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_ARTEC_SDK")
		MESSAGE(STATUS "Found ARTEC SDK, adding to current build")
	endif()

	target_include_directories(${COMPONENT_NAME} PUBLIC ${ARTEC_INCLUDE_DIR})
	target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${ARTEC_LIBS})
endif()

#########################################
# catking
if(${MICO_USE_ROS})
	find_package(catkin COMPONENTS roscpp tf rosbag cv_bridge image_transport sensor_msgs geometry_msgs)
	if(${catkin_FOUND})
		catkin_package(
			CATKIN_DEPENDS roscpp tf rosbag cv_bridge image_transport sensor_msgs geometry_msgs
			INCLUDE_DIRS include
		)
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "MICO_USE_ROS")

		target_include_directories(${COMPONENT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
		target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC ${catkin_LIBRARIES} )
		add_dependencies(${COMPONENT_NAME} ${catkin_EXPORTED_TARGETS})
		
		target_include_directories(${COMPONENT_NAME}  PUBLIC ${catkin_INCLUDE_DIRS})
		target_link_libraries(${COMPONENT_NAME}  LINK_PUBLIC  ${catkin_LIBRARIES})
		target_include_directories(${COMPONENT_NAME}  PUBLIC ${rosbag_INCLUDE_DIRS})
		target_link_libraries(${COMPONENT_NAME}  LINK_PUBLIC  ${rosbag_LIBRARIES})
		MESSAGE(STATUS "Found ROS Catkin")
	else()
		set(MICO_USE_ROS OFF)
	endif()
endif(${MICO_USE_ROS})

#########################################
# MAVLINK SDK to communicate with PX4 or ardupilot
if (${USE_MAVLINK})
	find_package(MAVSDK)
	if(${MAVSDK_FOUND})
		target_compile_definitions(${COMPONENT_NAME} PUBLIC "HAS_MAVSDK")	
		target_link_libraries(${COMPONENT_NAME} LINK_PUBLIC MAVSDK::mavsdk_telemetry MAVSDK::mavsdk_action MAVSDK::mavsdk )
	else()
		set(USE_MAVLINK OFF)
	endif()
endif()

#########################################
######			Library 		   ######
#########################################
#add_definitions(-fopenmp)

#########################################
######			Documentation	   ######
#########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_rgbd_tools
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

#########################################
######		   	Tools 		   	   ######
#########################################
if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			Examples 		   ######
#########################################
if(${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif(${BUILD_EXAMPLES})


#########################################
######			Examples 		   ######
#########################################
if(${BUILD_TESTS})
  find_package(GTest)
  if(${GTest_FOUND})
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()


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

# DESTINATION will be automatically prefixed by ${CMAKE_INSTALL_PREFIX}
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  COMPONENT ${COMPONENT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
