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

#include <mico/flow/blocks/processors/BlockOdometryLidar.h>

#include <flow/Policy.h>
#include <flow/Outpipe.h>

namespace mico{

    BlockOdometryLidar::BlockOdometryLidar(){
        
        createPolicy({{"Lidar PointCloud","cloud"}});
        createPipe("Estimated Dataframe" , "dataframe");

        
        registerCallback({"Lidar PointCloud"}, 
                            [&](flow::DataFlow _data){
                                if(idle_){
                                    idle_ = false;
                                    std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> df(new Dataframe<pcl::PointXYZRGBNormal>(nextDfId_));
                                    nextDfId_++;
                                    df->cloud(_data.get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("Lidar PointCloud")); 
                                    if(prevDf_){
                                        Eigen::Matrix4f initPose = prevDf_->pose();
                                        if(!icpAlignment<pcl::PointXYZRGBNormal, DebugLevels::Debug, OutInterfaces::Cout>(df->cloud(),
                                                            prevDf_->cloud(),
                                                            initPose,
                                                            5,     // random
                                                            2.0,    // m
                                                            -1,
                                                            -1.0,
                                                            100.0,
                                                            100.0,
                                                            1000,   // RANDOM, too high
                                                            0.3))  // RANDOM, too high
                                        {
                                            // ERROR
                                            std::cout << "ICP failed" << std::endl;
                                        }
                                        else{
                                            df->pose(initPose);
                                            memoryDf_[df->id()] = df;   // 666 safety reasons, but memory consumption.
                                            getPipe("Estimated Dataframe")->flush(df);  
                                        }
                                    }

                                    prevDf_ = df;
                                    idle_ = true;
                                }
                            });

    }


    bool BlockOdometryLidar::configure(std::unordered_map<std::string, std::string> _params){
        // for(auto &param: _params){
        //     if(param.first == "something" && param.second != ""){

        //     }
        // }

        return true;

    }
    
    std::vector<std::string> BlockOdometryLidar::parameters(){
        return {};
    }

}
