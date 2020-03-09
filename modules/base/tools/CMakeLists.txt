macro(build_tool tool_name tool_files)
    add_executable(${tool_name} ${tool_files})
    target_include_directories(${tool_name} PRIVATE ../include)
    target_link_libraries(${tool_name} LINK_PRIVATE mico::mico-base)
	## Load optitrack
	if(${USE_OPTITRACK})
		find_package(optitrack_flexible_framework REQUIRED) 
		target_compile_definitions(${tool_name} PUBLIC "ENABLE_OPTITRACK")
		target_link_libraries(${tool_name} LINK_PUBLIC optitrack_flexible_framework::optitrack_flexible_framework)
	endif()
	if (${USE_DARKNET})
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet/include")
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet/src")
	endif(${USE_DARKNET})
	if (${USE_DARKNET_CL})
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet_cl/darknet_cl/include")
		target_include_directories(${tool_name} PRIVATE "../ThirdParty/darknet_cl/darknet_cl/src")
	endif(${USE_DARKNET_CL})
endmacro()

build_tool(dataset_recorder dataset_recorder/main.cpp)

#build_tool(feature_object_tracking feature_object_tracking/main.cpp)
#build_tool(feature_cloud_extractor feature_cloud_extractor/main.cpp)

# find_package(catkin COMPONENTS cv_bridge image_transport)
# if(${catkin_FOUND})

# 	catkin_package(
# 	  	CATKIN_DEPENDS roscpp
# 	)

# 	macro(build_ros_tool tool_name tool_files)
# 	    add_executable(${tool_name} ${tool_files})
#  	    target_include_directories(${tool_name} PUBLIC ../include)
# 	    target_link_libraries(${tool_name} LINK_PUBLIC rgbd_tools)
# 	    target_include_directories(${tool_name} PUBLIC ${catkin_INCLUDE_DIRS})
# 	    target_link_libraries(${tool_name} ${catkin_LIBRARIES})
# 	endmacro()

# 	# build_ros_tool(ros_publisher ros_publisher/main.cpp)

# endif(${catkin_FOUND})

