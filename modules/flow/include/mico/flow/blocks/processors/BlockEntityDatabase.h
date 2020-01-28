//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKENTITYDATABASE_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKENTITYDATABASE_H_

#include <flow/Block.h>
#include <mico/base/cjson/json.h>
#ifdef HAS_DARKNET
    #include <mico/dnn/map3d/Entity.h>
#endif

namespace mico{

    class BlockEntityDatabase: public flow::Block{
    public:
        static std::string name() {return "Entity Database";}

        BlockEntityDatabase();
        ~BlockEntityDatabase();
    
        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;        

    private:
        #ifdef HAS_DARKNET
            std::map<int, std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities_;
        #endif
        bool hasPrev_ = false;
        bool idle_ = true;
    };

}

#endif