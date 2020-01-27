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


#include <mico/flow/blocks/streamers/StreamWebcam.h>
#include <flow/Outpipe.h>

namespace mico{
        StreamWebcam::StreamWebcam(){
            createPipe("Color", "image");
        }


        StreamWebcam::~StreamWebcam(){
            if(camera_){
                camera_->release();
                delete camera_;
            } 
        };

        bool StreamWebcam::configure(std::unordered_map<std::string, std::string> _params) {
            if(isRunningLoop()) // Cant configure if already running.
                return false;            

            int deviceId = 0;
            for(auto &p:_params){
                if(p.first == "device_id")
                    deviceId = atoi(p.second.c_str());
            }
            
            camera_ = new cv::VideoCapture(deviceId);

            return camera_->isOpened();

        }
        
        std::vector<std::string> StreamWebcam::parameters(){
            return {
                "device_id" 
            };
        }

        void StreamWebcam::loopCallback() {
            while(isRunningLoop()){
                if(auto pipe = getPipe("Color"); pipe->registrations() !=0 ){
                    cv::Mat image;
                    (*camera_) >> image;
                    pipe->flush(image);     
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }         
        }
}