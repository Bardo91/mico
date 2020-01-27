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


#include <mico/flow/blocks/misc/BlocksUtils3D.h>

#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>

#include <pcl/common/transforms.h>
#include <Eigen/Eigen>

namespace mico{
    BlockTransformCloud::BlockTransformCloud(){
        createPipe("Transformed Cloud", "cloud");

        createPolicy({     {"Cloud","cloud"}, 
                            {"Transform","mat44"}});
        
        registerCallback({"Cloud", "Transform"}, 
                                [&](flow::DataFlow _data){
                                    if(getPipe("Transformed Cloud")->registrations() !=0 ){
                                        auto cloud = _data.get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("Cloud"); 
                                        auto transform = _data.get<Eigen::Matrix4f>("Transform"); 
                                        pcl::PointCloud<pcl::PointXYZRGBNormal> outCloud;
                                        pcl::transformPointCloudWithNormals(*cloud, outCloud, transform);
                                        getPipe("Transformed Cloud")->flush(outCloud.makeShared());
                                    }
                                });
    }

    BlockVoxelFiltering::BlockVoxelFiltering(){
        createPipe("Filtered Cloud", "cloud");
        voxeler_.setLeafSize (voxelSize_,voxelSize_,voxelSize_);

        createPolicy({{"Cloud","cloud"}});
        registerCallback({"Cloud"}, 
                                [&](flow::DataFlow _data){
                                        if(getPipe("Filtered Cloud")->registrations() !=0 ){
                                            auto cloud = _data.get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("Cloud"); 
                                            pcl::PointCloud<pcl::PointXYZRGBNormal> outCloud;
                                            voxeler_.setInputCloud (cloud);
                                            voxeler_.filter (outCloud);
                                            getPipe("Filtered Cloud")->flush(outCloud.makeShared());
                                        }
                                });
    }

    std::vector<std::string> BlockVoxelFiltering::parameters() {
        return {"voxel_size"};
    }

    bool BlockVoxelFiltering::configure(std::unordered_map<std::string, std::string> _params) {
        std::istringstream istr(_params["voxel_size"]);
        istr >> voxelSize_;
        voxeler_.setLeafSize (voxelSize_,voxelSize_,voxelSize_);
        return true;
    }
}
