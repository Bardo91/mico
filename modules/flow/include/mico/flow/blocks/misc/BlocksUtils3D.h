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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKUTILS3D_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKUTILS3D_H_

#include <flow/Block.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace mico{
    class BlockTransformCloud: public flow::Block {
    public:
        virtual std::string name() const override {return "Transform Cloud";}

        BlockTransformCloud();

    private:
        bool idle_ = true;
    };

    class BlockVoxelFiltering: public flow::Block {
    public:
        virtual std::string name() const override {return "Voxel Filtering";}

        BlockVoxelFiltering();

        std::vector<std::string> parameters();
        bool configure(std::unordered_map<std::string, std::string> _params);

    private:
        bool idle_ = true;
        float voxelSize_ = 0.01;
        pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxeler_;
    };

}

#endif