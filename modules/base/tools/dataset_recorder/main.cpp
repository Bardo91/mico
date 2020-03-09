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

#include <string>
#include <unordered_map>

#include <mico/base/StereoCamera.h>
#include <mico/base/cjson/json.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

#ifdef ENABLE_OPTITRACK
    #include <Optitrack_Flexible_Framework/Optiwrap.h>
#endif

#ifdef _WIN32
#include <Windows.h>
    inline void do_mkdir(std::string _filename) {
        CreateDirectory(_filename.c_str(), NULL);
    }
#elif __linux__ || __APPLE__
#include <sys/stat.h>
#include <sys/types.h>
    inline void do_mkdir(std::string _filename) {
        mkdir(_filename.c_str(), 0700);
    }
#endif


int main(int _argc, char** _argv){
    #ifdef ENABLE_OPTITRACK
    Optiwrap optitrack;
    optitrack.set_IPs("192.168.1.103", "192.168.1.102");
    optitrack.start_communication();
    std::string name = "RS";
    int rigid_body_id = 666 ;
    bool check_rigid_body = true;
    #endif 
	cjson::Json mConfigFile;
	mico::StereoCamera *mCamera;

	std::string mStoreFolder;
	unsigned mStoreIndex = 0;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> mCloudViewer;
	std::ofstream mArmFile;
	Eigen::Vector3f mPrevCentroid;

	if (_argc != 2) {
            std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
            return -1;
        }

        std::ifstream file(_argv[1]);
        if (!file.is_open()) {
            std::cout << "Error opening config file" << std::endl;
            return -1;
        }

        if (!mConfigFile.parse(file)) {
            std::cout << "Error parsing config file" << std::endl;
            return -1;
        }

	// Instantiate camera
	mCamera = mico::StereoCamera::create((std::string) mConfigFile["cameraType"]);

	// Init camera
	if (mCamera == nullptr || !mCamera->init(mConfigFile["deviceConfig"])) {
	    std::cout << "Failed initialization of the camera" << std::endl;
	    return false;
	}

	// Create storage folder
	mStoreFolder = "dataset_" + std::to_string(time(NULL));
	do_mkdir(mStoreFolder);
    #ifdef ENABLE_OPTITRACK
    std::ofstream dataset("dataset_" + std::to_string(time(NULL)) + "/1optitrack.txt");
	#endif
    // Store calibration matrices
	cv::FileStorage fs(mStoreFolder+"/CalibrationFile.xml", cv::FileStorage::WRITE);
	cv::Mat matrixLeft, distCoefLeft, matrixRight, distCoefRight, rot, trans;

	if(mCamera->leftCalibration(matrixLeft, distCoefLeft)){
	    fs << "MatrixLeft" << matrixLeft;
	    fs << "DistCoeffsLeft" << distCoefLeft;
	}
	if(mCamera->rightCalibration(matrixRight, distCoefRight)){
	    fs << "MatrixRight" << matrixRight;
	    fs << "DistCoeffsRight" << distCoefRight;
	}
	if(mCamera->extrinsic(rot, trans)){
	    fs << "Rotation" << rot;
	    fs << "Translation" << trans;
	}
	double dispToDepth;
	if(mCamera->disparityToDepthParam(dispToDepth)){
	    fs << "DisparityToDepthScale" << dispToDepth;
	}

    fs.release();

	while(true){
		// Update index;
        mStoreIndex++;
        mCamera->grab();

        // Store data
        cv::Mat left, right, depth;
        if (mConfigFile["store"]["rgb"] && mCamera->rgb(left, right)) {
        
            if (left.rows != 0)
                cv::imwrite(mStoreFolder + "/left_" + std::to_string(mStoreIndex) + ".png", left);

            if (right.rows != 0)
                cv::imwrite(mStoreFolder + "/right_" + std::to_string(mStoreIndex) + ".png", right);
        }

        if (mConfigFile["store"]["depth"] && mCamera->depth(depth)) {
            if(depth.rows != 0)
                cv::imwrite(mStoreFolder + "/depth_" + std::to_string(mStoreIndex) + ".png", depth);
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
        pcl::PointCloud<pcl::PointXYZRGBNormal> colorNormalCloud;
        pcl::PointCloud<pcl::PointNormal> cloudNormal;
        if (mConfigFile["store"]["cloud"] == "xyz" && mCamera->cloud(cloud)) {
            pcl::io::savePCDFileBinaryCompressed(mStoreFolder + "/colorNormalCloud_" + std::to_string(mStoreIndex) + ".pcd", cloud);
        } else if (mConfigFile["store"]["cloud"] == "xyzrgb" && mCamera->cloud(colorCloud)) {
            pcl::io::savePCDFileBinaryCompressed(mStoreFolder + "/colorCloud_" + std::to_string(mStoreIndex) + ".pcd", colorCloud);
        } else if (mConfigFile["store"]["cloud"] == "xyzrgbn" && mCamera->cloud(colorNormalCloud)) {
            pcl::io::savePCDFileBinaryCompressed(mStoreFolder + "/cloud_" + std::to_string(mStoreIndex) + ".pcd", colorNormalCloud);
        } else if (mConfigFile["store"]["cloud"] == "normal" && mCamera->cloud(cloudNormal)) {
            pcl::io::savePCDFileBinaryCompressed(mStoreFolder + "/cloud_" + std::to_string(mStoreIndex) + ".pcd", cloudNormal);
        }
        #ifdef ENABLE_OPTITRACK
            optitrack.run(check_rigid_body, rigid_body_id, name);
            auto rb = optitrack.getRigidBody(rigid_body_id);
            if(rb){
                auto position = rb->position();
                Eigen::Quaternionf quat;
                rb->orientation(quat);
                dataset << mStoreIndex << " " << position.x() << " " << position.y() << " " << position.z()  << " " << quat.x() << " " 
                        << quat.y() << " " << quat.z() << " " << quat.w() << "\n";
            }
        #endif

	}
}
