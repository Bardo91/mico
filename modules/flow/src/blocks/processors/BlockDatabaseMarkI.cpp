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

#include <mico/flow/blocks/processors/BlockDatabaseMarkI.h>
#include <flow/Policy.h>
#include <flow/OutPipe.h>

#include <sstream>

namespace mico{

    BlockDatabaseMarkI::BlockDatabaseMarkI(){
        iPolicy_ = new flow::Policy({"dataframe"});

        opipes_["dataframe"] = new flow::OutPipe("dataframe");
        
        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> df = std::any_cast<std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>>(_data["dataframe"]);

                                        if(database_.addDataframe(df)){ // New dataframe created 
                                            opipes_["dataframe"]->flush(database_.lastDataframe());
                                        }
                                        idle_ = true;
                                    }
                                }
        );


    }

    BlockDatabaseMarkI::~BlockDatabaseMarkI(){

    } 


    bool BlockDatabaseMarkI::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.first =="vocabulary"){
                jParams["vocabulary"] = param.second;
            }
        }
        jParams["clusterComparison"] = 1;
        std::istringstream istr(_params["similarity_score"]);
        float similarityScore;
        istr >> similarityScore;
        jParams["similarity_score"] = similarityScore;

        return database_.init(jParams);
    }
    
    std::vector<std::string> BlockDatabaseMarkI::parameters(){
        return {"vocabulary", "similarity_score"};
    }
}
