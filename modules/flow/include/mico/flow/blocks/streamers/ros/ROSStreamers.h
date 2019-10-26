//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_
#define MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_

#include <mico/flow/blocks/streamers/ros/BlockROSSuscriber.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef MICO_USE_ROS
	#include <geometry_msgs/PoseStamped.h>
	#include <geometry_msgs/Pose.h>
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/Imu.h>
	#include <sensor_msgs/NavSatFix.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <cv_bridge/cv_bridge.h>
#endif

namespace mico{
	#ifdef MICO_USE_ROS
	struct TraitPoseStamped{
		static std::string blockName_;
		static std::vector<std::string> output_;
		static std::any conversion_(std::string _tag, const geometry_msgs::PoseStamped::ConstPtr &_msg);
		typedef geometry_msgs::PoseStamped RosType_;
	};
	struct TraitImu{
		static std::string blockName_;
		static std::vector<std::string> output_ ;
		typedef sensor_msgs::Imu RosType_;
		static std::any conversion_(std::string _tag, const sensor_msgs::Imu::ConstPtr &_msg);
	};
	struct TraitGPS{
		static std::string blockName_;
		static std::vector<std::string> output_ ;
		typedef sensor_msgs::NavSatFix RosType_;
		static std::any conversion_(std::string _tag, const sensor_msgs::NavSatFix::ConstPtr &_msg);
	};
	struct TraitImage{
		static std::string blockName_;
		static std::vector<std::string> output_;
		typedef sensor_msgs::Image RosType_;
		static std::any conversion_(std::string _tag, const sensor_msgs::Image::ConstPtr &_msg);
	};
	struct TraitCloud{
		static std::string blockName_;
		static std::vector<std::string> output_;
		typedef sensor_msgs::PointCloud2 RosType_;
		static std::any conversion_(std::string _tag, const sensor_msgs::PointCloud2::ConstPtr &_msg);
	};
	
	typedef BlockROSSuscriber< TraitPoseStamped > BlockRosPoseStamped;
	typedef BlockROSSuscriber< TraitCloud       > BlockRosCloud;
	typedef BlockROSSuscriber< TraitImu         > BlockRosImu;
	typedef BlockROSSuscriber< TraitGPS         > BlockRosGPS;
	typedef BlockROSSuscriber< TraitImage       > BlockRosImage;			

	#endif
}

#endif