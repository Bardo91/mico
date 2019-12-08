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

#include <mico/flow/blocks/processors/BlockLoopClosure.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>

#include <sstream>

namespace mico{

    BlockLoopClosure::BlockLoopClosure(){
        createPolicy({{"Next Keyframe", "dataframe"}});

        createPipe("Loop","v-dataframe");

        registerCallback({"Next Keyframe"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        Dataframe<pcl::PointXYZRGBNormal>::Ptr df = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("dataframe"); 
                                        
                                        cv::Mat image = df->leftImage();
                                        LoopResult res = loopDetector_.appendCluster(image, df->id());
                                        dataframes_[df->id()] = df;

                                        if(res.found){ // New dataframe created 
                                            // std::cout << "Detected loop: " << std::endl;
                                            
                                            auto loopPath = loopDetector_.findPath(df, dataframes_[res.matchId]);
                                            // std::cout << "Path: ";
                                            // for(auto &df: loopPath){
                                            //     std::cout << df->id() << "->";
                                            // }
                                            // std::cout << std::endl;

                                            // mDatabase.dfComparison(loopClosureSubset, false);  666 do it or not? @Ric92
                                            
                                            getPipe("loop")->flush(loopPath);

                                        }
                                        idle_ = true;
                                    }
                                }
        );


    }

    BlockLoopClosure::~BlockLoopClosure(){

    } 


    bool BlockLoopClosure::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.first =="vocabulary"){
                if(param.second == "")
                    return false;
                    
                jParams["vocabulary"] = param.second;
            }
        }

        return loopDetector_.init(jParams);
    }
    
    std::vector<std::string> BlockLoopClosure::parameters(){
        return {"vocabulary"};
    }
}
