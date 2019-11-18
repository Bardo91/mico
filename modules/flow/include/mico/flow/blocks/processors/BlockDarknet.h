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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKDARKNET_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKDARKNET_H_

#include <mico/flow/Block.h>
#include <mico/base/map3d/Dataframe.h>
#ifdef HAS_DARKNET
    #include <mico/dnn/object_detection/dnn/WrapperDarknet.h>
    #include <mico/dnn/map3d/Entity.h>
    #include <pcl/filters/filter.h>
#endif
#include <experimental/filesystem>

namespace mico{

    class BlockDarknet: public Block{
    public:
        static std::string name() {return "Darknet";}

        BlockDarknet();
        // ~BlockDarknet(){};

        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

    private:
        bool idle_ = true;
        bool hasParameters_ = false; //weights, cfg, confidence threshold and use dense cloud
        float confidenceThreshold = 0.7;
        int numEntities_ = 0;
        bool useDenseCloud_ = false;
        #ifdef HAS_DARKNET
        mico::WrapperDarknet detector_;
        #endif
    };

}

#endif