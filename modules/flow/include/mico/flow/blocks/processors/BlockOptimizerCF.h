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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKOPTIMIZERCF_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKOPTIMIZERCF_H_

#include <flow/Block.h>
#include <mico/base/map3d/BundleAdjuster_g2o.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mico{

    class BlockOptimizerCF: public flow::Block{
    public:
        virtual std::string name() const override {return "Optimizer CFs (g2o)";}

        BlockOptimizerCF();
        // ~BlockOptimizerCF(){};
    
        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

        std::string description() const override {return    "Block for optimizing 3D graph maps using g2o.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};
    private:
        bool idle_ = true;
        mico::BundleAdjuster_g2o<pcl::PointXYZRGBNormal>/*, DebugLevels::Debug, OutInterfaces::Cout>*/ optimizer_;
    };

}

#endif