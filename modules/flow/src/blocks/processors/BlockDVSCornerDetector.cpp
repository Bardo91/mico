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

#include <mico/flow/blocks/processors/BlockDVSCornerDetector.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>

#include <sstream>

namespace mico{

    BlockDVSCornerDetector::BlockDVSCornerDetector(){
        createPipe("Corners events", "v-event");
        
        createPolicy({{"Unfiltered events", "v-event"}});
        registerCallback({"Unfiltered events"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        auto UnfiltEvents = _data.get<dvs_msgs::EventArray>("Unfiltered events");
                                        detector_->eventCallback(UnfiltEvents);
                                        dvs_msgs::EventArray corners = detector_->cornersDetected();
                                        
                                        if(corners.events.size() > 0){
                                            getPipe("Corners events")->flush(corners);
                                        }
                                        idle_ = true;
                                    }
                                }
        );


    }

    BlockDVSCornerDetector::~BlockDVSCornerDetector(){

    } 


    bool BlockDVSCornerDetector::configure(std::unordered_map<std::string, std::string> _params){
        std::string detecMethod;
        for(auto &param: _params){
            if(param.second == "")
                return false;

            if(param.first == "Detection_method"){
                detecMethod = param.second;
            }
        }
        if (detecMethod == "harris"){
            detector_ = new mico::HarrisDetector;
        }
        else if (detecMethod == "fast"){
            detector_ = new mico::FastDetector;
        }else{
            return false;
        }

        return true;
    }
    
    std::vector<std::string> BlockDVSCornerDetector::parameters(){
        return {"Detection_method"};
    }
}
