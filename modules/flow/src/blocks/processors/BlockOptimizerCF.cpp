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
#include <flow/Policy.h>
#include <flow/Outpipe.h>

#include <sstream>

namespace mico{

    BlockOptimizerCF::BlockOptimizerCF(){
        createPolicy({{"Subset Dataframes","v-dataframe"}});
        
        registerCallback({"Subset Dataframes"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        // std::cout << "Optimization start" << std::endl;
                                        auto vDataframes = _data.get<std::vector<Dataframe<pcl::PointXYZRGBNormal>::Ptr>>("Subset Dataframes");
                                        std::map<int, Dataframe<pcl::PointXYZRGBNormal>::Ptr> dfMap;
                                        for(auto &df: vDataframes){
                                            for(auto &visDf: df->covisibility()){
                                                dfMap[visDf->id()] = visDf;
                                            }
                                        }
                                        // std::cout << "Loop of size: " << vDataframes.size() << ". Total optimized frames: " << dfMap.size() << std::endl; 
                                        optimizer_.sequence(dfMap);
                                        optimizer_.optimize();
                                        // std::cout << "Optimization end" << std::endl;
                                        idle_ = true;
                                    }
                                }
        );


    }

    bool BlockOptimizerCF::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.second == "")
                    return false;
            
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
