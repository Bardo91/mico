////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAREALSENSE_H_
#define RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAREALSENSE_H_

#include "../StereoCamera.h"

#include <cjson/json.h>

// Forward declarations
namespace rs {
	class context;
	class device;

	struct extrinsics;
	struct intrinsics;
}

namespace rgbd {
	/// Wrapper for IntelReal Sense 3D camera.
	class StereoCameraRealSense :public StereoCamera {
	public:		// Public interface
		/// \brief Initialize the camera using a config file.
		/// Config file must have following structure.
		///
		/// \code
		///     {
		///			"syncStreams":true|false,
		///			"depth":
		///				{
		///					"resolution":640,	// Desired resolution, if didn't exist, take the closer one
		///					"fps":30			// Desired fps, if didn't exist, take the closer one
		///				},
		///			"rgb":
		///				{
		///					"resolution":640,	// Desired resolution, if didn't exist, take the closer one
		///					"fps":30			// Desired fps, if didn't exist, take the closer one
		///				}	
		///			"useUncolorizedPoints":true|false	// The FOV of the Depth camera is larger than the color one.
		///												// Set true to use them in the color cloud even it they dont 
		///												// color, or set it to false to disable them and take only the
		///												// colorized points.
		///     }
		/// \endcode
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief get the rgb frame and fill only the left image.
		bool rgb(cv::Mat &_left, cv::Mat &_right, bool _undistort = true);

		/// \brief get the depth frame generated by the IR sensors. 
		bool depth(cv::Mat &_depth);

		/// \brief Grab current data from camera to make sure that is synchronized.
		bool grab();

		/// \brief Get a new point cloud from the camera with only spatial information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial, surface normals and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial information and surface normals.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

	private:	//	Private interface
		template<typename PointType_>
		bool setOrganizedAndDense(pcl::PointCloud<PointType_> &_cloud);

	private:	// Private members
		cjson::Json mConfig;

		rs::context *mRsContext;
		rs::device *mRsDevice;

		rs::intrinsics *mRsDepthIntrinsic;
		rs::extrinsics *mRsDepthToColor;
		rs::intrinsics *mRsColorIntrinsic;
		float mRsDepthScale;


		bool mUseUncolorizedPoints = false;
		bool mHasRGB = false, mComputedDepth = false;
		cv::Mat mLastDepth, mLastRGB;
	};	//	class StereoCameraRealSense


	template<typename PointType_>
	inline bool StereoCameraRealSense::setOrganizedAndDense(pcl::PointCloud<PointType_> &_cloud) {
		if (mHasRGB && mComputedDepth) {
			_cloud.is_dense = false;
			_cloud.width = mLastDepth.cols;
			_cloud.height = mLastDepth.rows;
			return true;
		}
		else {
			return false;
		}
	}

}	//	namespace RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAREALSENSE_H_

#endif