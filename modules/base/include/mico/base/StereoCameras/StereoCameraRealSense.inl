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



#include <pcl/features/integral_image_normal.h>

namespace mico {
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    bool StereoCameraRealSense::cloud(pcl::PointCloud<PointType_> &_cloud) {
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
            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, PointType_> ne;
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
                _cloud[i].rgb = ((int)cloudWoNormals[i].r) << 16 | ((int)cloudWoNormals[i].g) << 8 | ((int)cloudWoNormals[i].b);
                _cloud[i].r = cloudWoNormals[i].r;
                _cloud[i].g = cloudWoNormals[i].g;
                _cloud[i].b = cloudWoNormals[i].b;
            }

            return true;
        }
    }
}
