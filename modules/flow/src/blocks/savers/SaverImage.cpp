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

#include <mico/flow/blocks/savers/SaverImage.h>
#include <opencv2/opencv.hpp>

namespace mico{

    SaverImage::SaverImage(){
        iPolicy_ = new flow::Policy({"color", "depth"});

        iPolicy_->registerCallback({"color"}, 
                                [&](std::unordered_map<std::string,std::any> _data){                                
                                    counterGuardColor.lock();
                                    int id = idCounterColor;
                                    idCounterColor++;
                                    counterGuardColor.unlock();
                                    cv::Mat img = std::any_cast<cv::Mat>(_data["color"]);
                                    cv::imwrite(pathFolder_+"/color_"+std::to_string(id)+".png", img);
                                }
        );
        iPolicy_->registerCallback({"depth"}, 
                                [&](std::unordered_map<std::string,std::any> _data){                                
                                    counterGuardDepth.lock();
                                    int id = idCounterDepth;
                                    idCounterDepth++;
                                    counterGuardDepth.unlock();
                                    cv::Mat img = std::any_cast<cv::Mat>(_data["depth"]);
                                    cv::imwrite(pathFolder_+"/depth_"+std::to_string(id)+".png", img);
                                }
        );
    }
    
    bool SaverImage::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param:_params){
            if(param.first == "path_folder"){
                pathFolder_ = param.second;
                system(("mkdir -p "+ pathFolder_).c_str());   // 666 
                return true;
            }
        }
        
        return false;
    }

    std::vector<std::string> SaverImage::parameters(){
        return {"path_folder"};
    } 

}

