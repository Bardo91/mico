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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_PANGOLINVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_PANGOLINVISUALIZER_H_

#include <string>
#include <mutex>
#include <thread>
#include <functional>
#include <vector>

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/filters/voxel_grid.h>


namespace mico{

    #ifdef MICO_HAS_PANGOLIN
        class PangolinVisualizer {
        public:
            PangolinVisualizer();
            ~PangolinVisualizer();

            void currentPose(const Eigen::Matrix4f &_pose);

            void addLine(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector4f &_color = {0.0f,1.0f,0.0f,0.6f});
            void addLines(const std::vector<Eigen::Vector3f> &_pts, const Eigen::Vector4f &_color = {0.0f,1.0f,0.0f,0.6f});

            void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);

            void clearPointClouds();
            void clearLines();

        private:
            void renderCallback();
            void drawLines();
            void drawPointClouds();
            void drawCurrentPose();

        private:
            bool running_ = true;
            std::string windowName_ = "";

            std::thread renderThread_;
            std::mutex renderGuard_;
            std::vector<std::vector<Eigen::Vector3f>> linesToDraw_;
            std::vector<Eigen::Vector4f> colorLines_;

            std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> cloudsToDraw_;
            pcl::PointCloud<pcl::PointXYZRGBNormal> absoluteCloud_;
            pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxelFilter_;

            Eigen::Matrix4f currentPose_;

            static int sWinId;
        };
    #endif

}

#endif