//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Marco Montes Grova (a.k.a marrcogrova)
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


#include <mico/flow/blocks/savers/SaverMapDatabase.h>

namespace mico{

    SaverMapDatabase::SaverMapDatabase(){
        createPolicy({{"Saved Keyframe", "dataframe"}});

       registerCallback({"Saved Keyframe"}, 
            [&](flow::DataFlow _data){
                if(idle_){
                    idle_ = false;
                    Dataframe<pcl::PointXYZRGBNormal>::Ptr df = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Saved Keyframe"); 
                    database_.update(df);

                    idle_ = true;
                }
            }
        );
    }
    
    SaverMapDatabase::~SaverMapDatabase(){
        // database_.saveAllDatabase();
    }

    bool SaverMapDatabase::configure(std::unordered_map<std::string, std::string> _params){
        std::string name;
        
        for(auto &param:_params){
            if(param.first == "database_name"){
                name = param.second;
            }
        }
        return database_.init(name , "save");
    }
    
    std::vector<std::string> SaverMapDatabase::parameters(){
        return {"database_name"}; //666 add uri?
    }

    
}