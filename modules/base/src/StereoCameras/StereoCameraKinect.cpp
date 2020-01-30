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


#include <mico/base/StereoCameras/StereoCameraKinect.h>

#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace mico {
// 666 CURRENT IMPLEMENTATION ONLY ALLOWS ONE INSTANTIATION OF THIS CLASS
bool StereoCameraKinect::mIsCurrentlyEnabled = false;
std::mutex StereoCameraKinect::mRgbMutex;
std::mutex StereoCameraKinect::mDepthMutex;
cv::Mat    StereoCameraKinect::mLastRGB;
cv::Mat    StereoCameraKinect::mLastDepthInColor;

//-----------------------------------------------------------------------------------------------------------------
StereoCameraKinect::~StereoCameraKinect() {
	#ifdef ENABLE_LIBFREENECT
        if(mFreenectDevice){
            freenect_stop_depth(mFreenectDevice);
            freenect_stop_video(mFreenectDevice);
            freenect_close_device(mFreenectDevice);
        }
        if(mFreenectContext){
            freenect_shutdown(mFreenectContext);
        }
	#endif
}

//-----------------------------------------------------------------------------------------------------------------
bool StereoCameraKinect::init(const cjson::Json & _json){
        #ifdef ENABLE_LIBFREENECT
			mConfig = _json;
            if(mIsCurrentlyEnabled){
                std::cout << "[STEREOCAMERA][KINECT] CURRENT IMPLEMENTATION ONLY ALLOWS ONE INSTATATION OF STEREOCAMERAKINECT CLASS!" << std::endl;
                return false;
            }
            mIsCurrentlyEnabled = true;
            // Init streams
            if (freenect_init(&mFreenectContext, NULL) < 0) {
                std::cout << "[STEREO CAMERA][KINECT] freenect_init() failed" << std::endl;
                return false;
            }

            freenect_select_subdevices(mFreenectContext, FREENECT_DEVICE_CAMERA);
            int num_devices = freenect_num_devices(mFreenectContext);
            if (num_devices <= 0) {
                std::cout << "[STEREOCAMERA][KINECT] No device found!" << std::endl;
                freenect_shutdown(mFreenectContext);
                return false;
            }
            // Open the first device.
            int ret = freenect_open_device(mFreenectContext, &mFreenectDevice, 0);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }

            ret = freenect_set_depth_mode(mFreenectDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }
            ret = freenect_set_video_mode(mFreenectDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }

            mLastRGB = cv::Mat(480,640, CV_8UC3);
            freenect_set_depth_callback(mFreenectDevice, depthCallback);
            freenect_set_video_callback(mFreenectDevice, rgbCallback);

            ret = freenect_start_depth(mFreenectDevice);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return false;
            }
            ret = freenect_start_video(mFreenectDevice);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return false;
            }

            mFreenectEventProcessor = std::thread([&](){    /// 666 MANAGE TO CLOSE THIS THREAD!
                while(mRunning &&  freenect_process_events(mFreenectContext) >= 0){}
            });


            if(mConfig.contains("calibFile") && std::string(mConfig["calibFile"]) != ""){
                mHasCalibration = true;

                cv::FileStorage fs((std::string)mConfig["calibFile"], cv::FileStorage::READ);

                fs["MatrixLeft"]            >> mMatrixLeft;
                fs["DistCoeffsLeft"]        >> mDistCoefLeft;
                fs["MatrixRight"]           >> mMatrixRight;
                fs["DistCoeffsRight"]       >> mDistCoefRight;
                fs["DisparityToDepthScale"] >> mDispToDepth;

            }else{
                mHasCalibration = false;
                // FILL CALIB WITH SOME STANDART VALUES
            }

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::rgb(cv::Mat & _left, cv::Mat & _right){
        #ifdef ENABLE_LIBFREENECT
            mRgbMutex.lock();
            mLastRGB.copyTo(_left);
            mRgbMutex.unlock();
            cv::cvtColor(_left, _left, CV_BGR2RGB);
            return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::depth(cv::Mat & _depth){
        #ifdef ENABLE_LIBFREENECT
            mDepthMutex.lock();
            mLastDepthInColor.copyTo(_depth);
            mDepthMutex.unlock();
            return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::grab(){
        #ifdef ENABLE_LIBFREENECT
            return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #ifdef ENABLE_LIBFREENECT
            double fx = mMatrixLeft.at<float>(0, 0);
            double fy = mMatrixLeft.at<float>(1, 1);
            double cx = mMatrixLeft.at<float>(0, 2);
            double cy = mMatrixLeft.at<float>(1, 2);
            for (int dy = 0; dy < mLastRGB.rows; dy++) {
                for (int dx = 0; dx < mLastRGB.cols; dx++) {
                    // Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
                    float depth_in_meters = depth_value/1000; //<-- 666 Scale into mm

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    pcl::PointXYZ point;
                    if (depth_value == 0) {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                    } else {
                        point.x = (dx-cx)*depth_in_meters/fx;
                        point.y = (dy-cy)*depth_in_meters/fy;
                        point.z = depth_in_meters;
                    }
                    _cloud.push_back(point);
                }
            }
            _cloud.is_dense = false;
            _cloud.width = mLastRGB.cols;
            _cloud.height = mLastRGB.rows;
			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
        #ifdef ENABLE_LIBFREENECT
            double fx = mMatrixLeft.at<float>(0, 0);
            double fy = mMatrixLeft.at<float>(1, 1);
            double cx = mMatrixLeft.at<float>(0, 2);
            double cy = mMatrixLeft.at<float>(1, 2);

            for (int dy = 0; dy < mLastRGB.rows; dy++) {
                for (int dx = 0; dx < mLastRGB.cols; dx++ ) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
                    float depth_in_meters = depth_value*mDispToDepth;
                    // Set invalid pixels with a depth value of zero, which is used to indicate no data
                    pcl::PointXYZRGB point;
                    if (depth_value == 0) {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                    } else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        point.x = (dx-cx)*depth_in_meters/fx;
                        point.y = (dy-cy)*depth_in_meters/fy;
                        point.z = depth_in_meters;
                        auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        point.r = rgb[0];
                        point.g = rgb[1];
                        point.b = rgb[2];
                    }

					_cloud.push_back(point);
                }
            }
            _cloud.is_dense = false;
            _cloud.width = mLastRGB.cols;
            _cloud.height = mLastRGB.rows;
            return true;

		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
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

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixLeft.copyTo(_intrinsic);
        mDistCoefLeft.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixRight.copyTo(_intrinsic);
        mDistCoefRight.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point) {
        return false;
    }
	#ifdef ENABLE_LIBFREENECT
		//---------------------------------------------------------------------------------------------------------------------
		void StereoCameraKinect::rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp) {
			mRgbMutex.lock();
			memcpy(mLastRGB.data, rgb , 640*480*3);
			mRgbMutex.unlock();
		}

		//---------------------------------------------------------------------------------------------------------------------
		void StereoCameraKinect::depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
			cv::Mat cvdepth = cv::Mat( 480, 640, CV_16UC1, depth);
			mDepthMutex.lock();
			cvdepth.copyTo(mLastDepthInColor);
			mDepthMutex.unlock();
		}
	#endif
}	//	namespace mico 
