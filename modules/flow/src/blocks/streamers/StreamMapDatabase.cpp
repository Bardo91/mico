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


#include <mico/flow/blocks/streamers/StreamMapDatabase.h>

namespace mico{

    StreamMapDatabase::StreamMapDatabase(){
        createPipe("Dataframe", "dataframe");
    }
    
    bool StreamMapDatabase::configure(std::unordered_map<std::string, std::string> _params){
        if(isRunningLoop()) // Cant configure if already running.
            return false;

        std::string name,db_path;
        for(auto &param:_params){
            if(param.first == "database_name"){
                name = param.second;
            }
            else if(param.first == "database_path"){
                db_path = param.second;
            }
        }
        if (!database_.init(name , "load"))
            return false;
        
        if (database_.restoreDatabaseFile(db_path)){
            database_.restoreDataframes(dataframesDb_);
            return true;
        }else{
            return false;
        }
    }
    
    std::vector<std::string> StreamMapDatabase::parameters(){
        return {"database_name" , "database_path"};
    }

    void StreamMapDatabase::loopCallback() {
        while(isRunningLoop()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // db freq

            if (idDb_ < dataframesDb_.size() - 1 ){
                if(getPipe("Dataframe")->registrations() !=0 ){
                    Dataframe<pcl::PointXYZRGBNormal>::Ptr dfToFlush = dataframesDb_[idDb_];
                    std::cout << dfToFlush->id() <<std::endl;
                    getPipe("Dataframe")->flush(dfToFlush);
                    idDb_++;
                }
            }else{
                std::cout << "Database empty \n";
            }
            
        }
    }      
}