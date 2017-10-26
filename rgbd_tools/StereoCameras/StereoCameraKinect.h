////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAKINECT_H_
#define RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAKINECT_H_

#include "../StereoCamera.h"

#include <cjson/json.h>

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>
#include <mutex>
#include <thread>

namespace rgbd {
	/// Wrapper for IntelReal Sense 3D camera.
    class StereoCameraKinect :public StereoCamera {
	public:		// Public interface
        ~StereoCameraKinect();
        /// \brief Initialize the camera using a config file. The coordinate system for the 3d is always on the color camera.
		/// Config file must have following structure.
		///
		/// \code
		///     {
        ///
		///     }
		/// \endcode
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief get the rgb frame and fill only the left image.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

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

        /// \brief templatized method to define the interface for retrieving a new point cloud from the camera with
        /// spatial, surface normals and RGB information (custom types out of PCL).
        /// \param _cloud: reference to a container for the point cloud.
        template<typename PointType_>
        bool cloud(pcl::PointCloud<PointType_> &_cloud);

        /// \brief get the calibration matrices of the left camera in opencv format. Matrices are CV_32F.
        virtual bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the calibration matrices of the depth (IR) camera in opencv format. Matrices are CV_32F.
        virtual bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the extrinsic matrices, i.e., transformation from left to depth (IR) camera. Matrices are CV_32F.
        virtual bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

        /// \brief get the extrinsic matrices, i.e., transformation from left to depth (IR) camera.
        virtual bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

        /// \brief get disparty-to-depth parameter typical from RGB-D devices.
        virtual bool disparityToDepthParam(double &_dispToDepth);

	private:	//	Private interface
        void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp);
        void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp);
	private:	// Private members
		cjson::Json mConfig;
        cv::Mat mLeft, mRight, mDepth;

        freenect_context *mFreenectContext;
        freenect_device *mFreenectDevice;
        std::mutex mRgbMutex, mDepthMutex;
        std::thread mFreenectEventProcessor;
        bool mRunning = true;

		bool mUseUncolorizedPoints = false;
		bool mHasRGB = false, mComputedDepth = false;
        cv::Mat mLastRGB, mLastDepthInColor;
	};	//	class StereoCameraRealSense

}	//	namespace rgbd

#include "StereoCameraKinect.inl"

#endif  // RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAREALSENSE_H_
