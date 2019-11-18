//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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


#include <mico/base/StereoCameras/StereoCameraRealSense.h>

#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace mico {
	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::init(const cjson::Json & _json){
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
			mConfig = _json;

            if(mConfig.contains("deviceId")){
                mDeviceId = mConfig["deviceId"];
            }

			// Initialize context and get device det device
            #if defined(ENABLE_LIBREALSENSE_V1)
				mRsContext = new rs::context();
				if (mRsContext->get_device_count() == 0) {
					std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
					return false;
				}
				mRsDevice = mRsContext->get_device(mDeviceId);
				std::cout << "[STEREOCAMERA][REALSENSE] Using device "<< mDeviceId <<", an "<< mRsDevice->get_name() << std::endl;
				std::cout << "[STEREOCAMERA][REALSENSE]     Serial number: " << mRsDevice->get_serial() << std::endl;
				std::cout << "[STEREOCAMERA][REALSENSE]     Firmware version: " << mRsDevice->get_firmware_version() << std::endl;
            #elif defined(ENABLE_LIBREALSENSE_V2)
				auto list = mRsContext.query_devices();
				if (list.size() == 0) {
					std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
					return false;
				}
                mRsDevice = list[mDeviceId];

                std::cout << "[STEREOCAMERA][REALSENSE] Using device "<< mDeviceId <<", an "<< mRsDevice.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
                std::cout << "[STEREOCAMERA][REALSENSE]     Serial number: " << mRsDevice.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
                std::cout << "[STEREOCAMERA][REALSENSE]     Firmware version: " << mRsDevice.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION)<< std::endl;
			
			#endif			

			if (mConfig.contains("cloudDownsampleStep")) {
				mDownsampleStep = mConfig["cloudDownsampleStep"];
			}

			// Initialize streams of data.
            #if defined(ENABLE_LIBREALSENSE_V1)
				mRsDevice->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
				mRsDevice->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
				mRsDevice->start();
            #elif defined(ENABLE_LIBREALSENSE_V2)
                rs2::config cfg;
                cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
                cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
                mRsPipeProfile = mRsPipe.start(cfg);

			#endif
			
			// Get intrinsics and extrinsics
            #if defined(ENABLE_LIBREALSENSE_V1)
				mRsDepthIntrinsic 	= mRsDevice->get_stream_intrinsics(rs::stream::depth);
				mRsDepthToColor 	= mRsDevice->get_extrinsics(rs::stream::depth, rs::stream::color);
				mRsColorToDepth 	= mRsDevice->get_extrinsics(rs::stream::color, rs::stream::depth);
				mRsColorIntrinsic 	= mRsDevice->get_stream_intrinsics(rs::stream::color);
				mRsDepthScale		= mRsDevice->get_depth_scale();

            #elif defined(ENABLE_LIBREALSENSE_V2)
                auto depth_stream = mRsPipeProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
                auto color_stream = mRsPipeProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

                mRsAlign = new rs2::align(RS2_STREAM_COLOR);

				mRsDepthToColor = depth_stream.get_extrinsics_to(color_stream);
				mRsColorToDepth = color_stream.get_extrinsics_to(depth_stream);
                mRsDepthIntrinsic = depth_stream.get_intrinsics();
                mRsColorIntrinsic = color_stream.get_intrinsics();

				auto sensor = mRsPipeProfile.get_device().first<rs2::depth_sensor>();
				mRsDepthScale = sensor.get_depth_scale();
			#endif

            // Projection matrix Depth
            mCvDepthIntrinsic = cv::Mat::eye(3,3,CV_32F);
            mCvDepthIntrinsic.at<float>(0,0) = mRsDepthIntrinsic.fx;
            mCvDepthIntrinsic.at<float>(1,1) = mRsDepthIntrinsic.fy;
            mCvDepthIntrinsic.at<float>(0,2) = mRsDepthIntrinsic.ppx;
            mCvDepthIntrinsic.at<float>(1,2) = mRsDepthIntrinsic.ppy;

            // Projection matrix Color
            mCvColorIntrinsic= cv::Mat::eye(3,3,CV_32F);
            mCvColorIntrinsic.at<float>(0,0) = mRsColorIntrinsic.fx;
            mCvColorIntrinsic.at<float>(1,1) = mRsColorIntrinsic.fy;
            mCvColorIntrinsic.at<float>(0,2) = mRsColorIntrinsic.ppx;
            mCvColorIntrinsic.at<float>(1,2) = mRsColorIntrinsic.ppy;

            mExtrinsicColorToDepth = cv::Mat::eye(4,4,CV_32F);
            cv::Mat(3,3,CV_32F, &mRsColorToDepth.rotation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(0,0,3,3)));
            mExtrinsicColorToDepth(cv::Rect(0,0,3,3)) = mExtrinsicColorToDepth(cv::Rect(0,0,3,3)).t(); // RS use color major instead of row mayor.
            cv::Mat(3,1,CV_32F, &mRsColorToDepth.translation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(3,0,1,3)));



            if(mConfig.contains("useUncolorizedPoints")){
			    mSetDenseCloud = (bool) mConfig["useUncolorizedPoints"];
            }

			// Other params
            #if defined(ENABLE_LIBREALSENSE_V1)
				if(mConfig.contains("others")){
					if(mConfig["others"].contains("r200_lr_autoexposure")){
						mRsDevice->set_option(rs::option::r200_lr_auto_exposure_enabled, mConfig["others"]["r200_lr_autoexposure"]?1.0f:0.0f);
					}
					if(mConfig["others"].contains("r200_emitter_enabled")){
						mRsDevice->set_option(rs::option::r200_emitter_enabled, mConfig["others"]["r200_emitter_enabled"]?1.0f:0.0f);
					}
					if(mConfig["others"].contains("r200_lr_exposure")){
						mRsDevice->set_option(rs::option::r200_lr_exposure, (int) mConfig["others"]["r200_lr_exposure"]);
					}
					if(mConfig["others"].contains("r200_lr_gain")){
						mRsDevice->set_option(rs::option::r200_lr_gain, (int) mConfig["others"]["r200_lr_gain"]);
					} 
				}

            #elif defined(ENABLE_LIBREALSENSE_V2)
				if(mConfig.contains("others")){
					std::cout << "[STEREOCAMERAS][REALSENSE]Custom parameters are not yet coded" << std::endl;
				}
			#endif

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::rgb(cv::Mat & _left, cv::Mat & _right){
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            mLastRGB.copyTo(_left);
			return mHasRGB;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::depth(cv::Mat & _depth){
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            mLastDepthInColor.copyTo(_depth);
			return mComputedDepth;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::grab(){
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            #if defined(ENABLE_LIBREALSENSE_V1)
				mRsDevice->wait_for_frames();

                cv::cvtColor(cv::Mat(mRsColorIntrinsic.height, mRsColorIntrinsic.width, CV_8UC3, (uchar*)mRsDevice->get_frame_data(rs::stream::color)), mLastRGB, CV_RGB2BGR);
				mHasRGB = true;

                mLastDepthInColor = cv::Mat(mRsDepthIntrinsic.height, mRsDepthIntrinsic.width, CV_16U, (uchar*) mRsDevice->get_frame_data(rs::stream::depth_aligned_to_color));
				mComputedDepth = true;

            #elif defined(ENABLE_LIBREALSENSE_V2)
                rs2::frameset frames = mRsPipe.wait_for_frames();

                auto processedFrames = mRsAlign->process(frames);

                rs2::frame frameDepth = processedFrames.first(RS2_STREAM_DEPTH);
                rs2::frame frameRGB = processedFrames.first(RS2_STREAM_COLOR);

                mLastRGB = cv::Mat(cv::Size(mRsColorIntrinsic.width, mRsColorIntrinsic.height), CV_8UC3, (void*)frameRGB.get_data(), cv::Mat::AUTO_STEP);
				mHasRGB = true;

                mLastDepthInColor = cv::Mat(cv::Size(mRsDepthIntrinsic.width, mRsDepthIntrinsic.height), CV_16U, (uchar*) frameDepth.get_data(), cv::Mat::AUTO_STEP);
				mComputedDepth = true;

			#endif

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)

        if(mSetDenseCloud){
            _cloud.resize(mLastDepthInColor.rows * mLastDepthInColor.cols);
            setOrganizedAndDense(_cloud);
        }

        for (int dy = 0; dy < mLastDepthInColor.rows; dy = dy + mDownsampleStep) {
            for (int dx = 0; dx < mLastDepthInColor.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    if (depth_value == 0) {
                        if (mSetDenseCloud) {
                            _cloud.push_back(pcl::PointXYZ(NAN, NAN, NAN));
                        }
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        cv::Point2f depth_pixel(dx, dy);
                        cv::Point3f depth_point = deproject(depth_pixel, depth_in_meters);

                        if(mSetDenseCloud)
                            _cloud.at(dx, dy) = pcl::PointXYZ(depth_point.x, depth_point.y, depth_point.z);
                        else
	    					_cloud.push_back(pcl::PointXYZ(depth_point.x, depth_point.y, depth_point.z));
					}
				}
			}
            

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            for (int dy = 0; dy < mLastDepthInColor.rows; dy = dy + mDownsampleStep) {
                for (int dx = 0; dx < mLastDepthInColor.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;
                    // Set invalid pixels with a depth value of zero, which is used to indicate no data
                    pcl::PointXYZRGB point;
                    if (depth_value == 0) {
                        if (mSetDenseCloud) {
                            point.x = NAN;
                            point.y = NAN;
                            point.z = NAN;
                        }
                        else
                            continue;
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        cv::Point2f depth_pixel(dx, dy);
                        cv::Point3f depth_point = deproject(depth_pixel, depth_in_meters);
                        point.x = depth_point.x;
                        point.y = depth_point.y;
                        point.z = depth_point.z;
                        auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        point.r = rgb[2];
                        point.g = rgb[1];
                        point.b = rgb[0];

                    }

					_cloud.push_back(point);
                }
            }
            if(mSetDenseCloud)
				setOrganizedAndDense(_cloud);
            return true;

		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
		if (!mSetDenseCloud) {
			std::cout << "[STEREOCAMERA][REALSENSE] Cannot compute the normals if points out of the colorized region are ignored. Please set the \"UseUncolorizedPoints\" to true in the configuration of the camera" << std::endl;
			return false;
		}
		else {
			pcl::PointCloud<pcl::PointXYZRGB> cloudWoNormals;
			if (!cloud(cloudWoNormals)) {
				return false;
			}

            if(cloudWoNormals.size() == 0){
                std::cout << "[STEREOCAMERA][REALSENSE] Empty cloud, can't compute normals" << std::endl;
                _cloud.resize(0);
                return true;
            }

			//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			ne.setInputCloud(cloudWoNormals.makeShared());
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(10.0f);
			ne.compute(_cloud);

			// Fill XYZ and RGB of cloud
			for (unsigned i = 0; i < _cloud.size(); i++) {
				_cloud[i].x = cloudWoNormals[i].x;
				_cloud[i].y = cloudWoNormals[i].y;
				_cloud[i].z = cloudWoNormals[i].z;
				_cloud[i].r = cloudWoNormals[i].r;
				_cloud[i].g = cloudWoNormals[i].g;
				_cloud[i].b = cloudWoNormals[i].b;
			}

			return true;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
			mCvColorIntrinsic.copyTo(_intrinsic);
            _coefficients = cv::Mat(1,5, CV_32F, mRsColorIntrinsic.coeffs);
			return true;
		#else
			return false;
		#endif
	}

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
			mCvDepthIntrinsic.copyTo(_intrinsic);
            _coefficients = cv::Mat(1,5, CV_32F, mRsColorIntrinsic.coeffs);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            cv::Mat(3,3,CV_32F, &mRsDepthToColor.rotation[0]).copyTo(_rotation);
            cv::Mat(3,1,CV_32F, &mRsDepthToColor.translation[0]).copyTo(_translation);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            _rotation = Eigen::Matrix3f(&mRsDepthToColor.rotation[0]);
            _translation = Eigen::Vector3f(&mRsDepthToColor.translation[0]);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mRsDepthScale;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = mLastDepthInColor.at<uint16_t>(_pixel.y, _pixel.x);
			float depth_in_meters = depth_value * mRsDepthScale;
			// Set invalid pixels with a depth value of zero, which is used to indicate no data
			pcl::PointXYZRGB point;
			if (depth_value == 0) {
				return false;
			}
			else {
				// Map from pixel coordinates in the depth image to pixel coordinates in the color image
                cv::Point2f depth_pixel(_pixel.x, _pixel.y);
                cv::Point3f depth_point = deproject(depth_pixel, depth_in_meters);

				_point.x = depth_point.x;
				_point.y = depth_point.y;
				_point.z = depth_point.z;
				return true;
			}
        #else
			return false;
		#endif 
    }

	//----------------------------------------------------------------------------------------------------------------- 
    bool StereoCameraRealSense::laserPower(double power_level){ 
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            #if defined(ENABLE_LIBREALSENSE_V1)
				mRsDevice->set_option(rs::option::f200_laser_power, power_level); 

            #elif defined(ENABLE_LIBREALSENSE_V2)
                auto depthSensor = mRsDevice.first<rs2::depth_sensor>();
				if (depthSensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, power_level > 0? 1: 0);
				} else if (depthSensor.supports(RS2_OPTION_LASER_POWER)) {
					auto range = depthSensor.get_option_range(RS2_OPTION_LASER_POWER);
					depthSensor.set_option(RS2_OPTION_LASER_POWER, power_level > range.max? range.max : power_level);
				}
				
			#endif
			
            return true;
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------

    #if defined(ENABLE_LIBREALSENSE_V1)
    cv::Point StereoCameraRealSense::distortPixel(const cv::Point &_point, const rs::intrinsics &_intrinsics) const {
    #elif defined(ENABLE_LIBREALSENSE_V2)
    cv::Point StereoCameraRealSense::distortPixel(const cv::Point &_point, const rs2_intrinsics &_intrinsics) const {
    #endif
    #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            float x = (_point.x - _intrinsics.ppx) / _intrinsics.fx;
            float y = (_point.y - _intrinsics.ppy) / _intrinsics.fy;

            float r2  = x*x + y*y;
            float f = 1 + _intrinsics.coeffs[0]*r2 + _intrinsics.coeffs[1]*r2*r2 + _intrinsics.coeffs[4]*r2*r2*r2;
            x *= f;
            y *= f;
            float dx = x + 2*_intrinsics.coeffs[2]*x*y + _intrinsics.coeffs[3]*(r2 + 2*x*x);
            float dy = y + 2*_intrinsics.coeffs[3]*x*y + _intrinsics.coeffs[2]*(r2 + 2*y*y);
            x = dx;
            y = dy;

            cv::Point distortedPixel;
            distortedPixel.x = x * _intrinsics.fx + _intrinsics.ppx;
            distortedPixel.y = y * _intrinsics.fy + _intrinsics.ppy;

            return distortedPixel;
    }
    #endif

    //---------------------------------------------------------------------------------------------------------------------
    #if defined(ENABLE_LIBREALSENSE_V1)
    cv::Point StereoCameraRealSense::undistortPixel(const cv::Point &_point,  const rs::intrinsics &_intrinsics) const {
    #elif defined(ENABLE_LIBREALSENSE_V2)
    cv::Point StereoCameraRealSense::undistortPixel(const cv::Point &_point, const rs2_intrinsics &_intrinsics) const {
    #endif
    #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            float x = (_point.x - _intrinsics.ppx) / _intrinsics.fx;
            float y = (_point.y - _intrinsics.ppy) / _intrinsics.fy;

            float r2  = x*x + y*y;
            float f = 1 + _intrinsics.coeffs[0]*r2 + _intrinsics.coeffs[1]*r2*r2 + _intrinsics.coeffs[4]*r2*r2*r2;
            float ux = x*f + 2*_intrinsics.coeffs[2]*x*y + _intrinsics.coeffs[3]*(r2 + 2*x*x);
            float uy = y*f + 2*_intrinsics.coeffs[3]*x*y + _intrinsics.coeffs[2]*(r2 + 2*y*y);

            cv::Point undistortedPixel;
            undistortedPixel.x = ux* _intrinsics.fx + _intrinsics.ppx;;
            undistortedPixel.y = uy* _intrinsics.fy + _intrinsics.ppy;;

            return undistortedPixel;
    }
    #endif

    //---------------------------------------------------------------------------------------------------------------------
    cv::Point3f StereoCameraRealSense::deproject(const cv::Point &_point, const float _depth) const {
        #if defined(ENABLE_LIBREALSENSE_V1) || defined(ENABLE_LIBREALSENSE_V2)
            float x = (_point.x - mRsDepthIntrinsic.ppx) / mRsDepthIntrinsic.fx;
            float y = (_point.y - mRsDepthIntrinsic.ppy) / mRsDepthIntrinsic.fy;

            cv::Point3f p(x*_depth, y*_depth, _depth);

            return p;
        #else
            return cv::Point3f();
        #endif
    }

}	//	namespace mico 
