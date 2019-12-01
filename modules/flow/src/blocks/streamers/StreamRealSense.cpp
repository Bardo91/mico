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


#include <mico/flow/blocks/streamers/StreamRealSense.h>
#include <flow/Outpipe.h>

namespace mico{

        StreamRealSense::StreamRealSense(){
            opipes_["Color"] = new flow::Outpipe("Color", "image");
            opipes_["Depth"] = new flow::Outpipe("Depth", "image");
            opipes_["Cloud"] = new flow::Outpipe("Cloud", "cloud");
        }

        bool StreamRealSense::configure(std::unordered_map<std::string, std::string> _params) {
            if(runLoop_ || hasInitCamera_) // Cant configure if already running.
                return true;

            cjson::Json jParams = {};
            for(auto &p:_params){
                if(p.first == "devide_id"){
                    jParams["deviceId"] = atoi(p.second.c_str());
                }else if(p.first == "cloudDownsampleStep"){
                    jParams["cloudDownsampleStep"] = atoi(p.second.c_str());
                }else if(p.first == "useUncolorizedPoints"){
                    jParams["useUncolorizedPoints"] = p.second == "true" ? true : false;
                }
            }

            hasInitCamera_ = camera_.init(jParams);
            
            return hasInitCamera_;
        }
        
        std::vector<std::string> StreamRealSense::parameters(){
            return {
                "devide_id" 
            };
        }

        void StreamRealSense::loopCallback() {
            if(!hasInitCamera_){
                cjson::Json dummy;  // 666 do it better
                dummy["dummy"] = "dummy";
                camera_.init(dummy);
            }
                
            for(unsigned i = 0; i < 10; i++){
                camera_.grab(); // 666 Grab some images to remove trash initial ones
            }

            while(runLoop_){
                cv::Mat left, right, depth;
                pcl::PointCloud<pcl::PointXYZRGBNormal> colorNormalCloud;
                camera_.grab();
                if(opipes_["Color"]->registrations() !=0 ){
                    camera_.rgb(left, right);
                    opipes_["Color"]->flush(left);     
                }
                if(opipes_["Depth"]->registrations() !=0 ){
                    camera_.depth(depth);
                    opipes_["Depth"]->flush(depth);
                }
                if(opipes_["Cloud"]->registrations() !=0 ){
                    camera_.cloud(colorNormalCloud);
                    opipes_["Cloud"]->flush(colorNormalCloud.makeShared());
                }
            }      
        }
}