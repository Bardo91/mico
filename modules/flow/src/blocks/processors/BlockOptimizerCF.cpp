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

#include <mico/flow/blocks/processors/BlockOptimizerCF.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

#include <sstream>

namespace mico{

    BlockOptimizerCF::BlockOptimizerCF(){
        iPolicy_ = new Policy({"v-clusterframes"});
        
        iPolicy_->setCallback({"v-clusterframes"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        // std::cout << "Optimization start" << std::endl;
                                        idle_ = false;
                                        auto vclusters = std::any_cast<std::vector<ClusterFrames<pcl::PointXYZRGBNormal>::Ptr>>(_data["v-clusterframes"]);
                                        std::map<int, ClusterFrames<pcl::PointXYZRGBNormal>::Ptr> clustersmap;
                                        for(auto &cf: vclusters){
                                            clustersmap[cf->id] = cf;
                                        }
                                        optimizer_.clusterframes(clustersmap);
                                        optimizer_.optimizeClusterframes();
                                        idle_ = true;
                                        // std::cout << "Optimization end" << std::endl;
                                    }
                                }
        );


    }

    bool BlockOptimizerCF::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first =="min_error"){
                std::istringstream istr(_params["min_error"]);
                float minError;
                istr >> minError;
                optimizer_.minError(minError);
            }else if(param.first =="iterations"){
                optimizer_.iterations(atoi(_params["iterations"].c_str()));
            }else if(param.first =="min_aparitions"){
                optimizer_.minAparitions(atoi(_params["min_aparitions"].c_str()));
            }else if(param.first =="min_words"){
                optimizer_.minWords(atoi(_params["min_words"].c_str()));
            }
        }


        return true;
    }
    
    std::vector<std::string> BlockOptimizerCF::parameters(){
        return {"min_error", "iterations", "min_aparitions", "min_words"};
    }
}
