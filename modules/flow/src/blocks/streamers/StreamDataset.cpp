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


#include <mico/flow/blocks/streamers/StreamDataset.h>
#include <flow/Outpipe.h>

namespace mico{
        StreamDataset::StreamDataset(){
            opipes_["color"] = new flow::Outpipe("color");
            opipes_["depth"] = new flow::Outpipe("depth");
            opipes_["cloud"] = new flow::Outpipe("cloud");
        }

        bool StreamDataset::configure(std::unordered_map<std::string, std::string> _params) {
            if(runLoop_) // Cant configure if already running.
                return false;            

            cjson::Json jParams;
            for(auto &p:_params){
                if(p.first == "color"){
                    jParams["input"]["left"] = p.second; // 666 param....
                }else if(p.first == "right"){
                    jParams["input"]["right"] = p.second;
                }else if(p.first == "depth"){
                    jParams["input"]["depth"] = p.second;
                }else if(p.first == "pointCloud"){
                    jParams["input"]["pointCloud"] = p.second;
                }else if(p.first == "firstIdx"){
                    jParams["firstIdx"] = atoi(p.second.c_str());
                }else if(p.first == "stepIdx"){
                    jParams["stepIdx"] = atoi(p.second.c_str());
                }else if(p.first == "loop_dataset"){
                    jParams["loop_dataset"] = atoi(p.second.c_str());
                }else if(p.first == "calibration"){
                    jParams["calibFile"] = p.second;
                }
            }
            return camera_.init(jParams);

        }
        
        std::vector<std::string> StreamDataset::parameters(){
            return {
                "color", "depth", "calibration" 
            };
        }

        void StreamDataset::loopCallback() {
            while(runLoop_){
                cv::Mat left, right, depth;
                pcl::PointCloud<pcl::PointXYZRGBNormal> colorNormalCloud;
                camera_.grab();
                if(opipes_["color"]->registrations() !=0 ){
                    if(camera_.rgb(left, right) && left.rows != 0)
                        opipes_["color"]->flush(left);     
                }
                if(opipes_["depth"]->registrations() !=0 ){
                    if(camera_.depth(depth) && depth.rows != 0)
                        opipes_["depth"]->flush(depth);
                }
                if(opipes_["cloud"]->registrations() !=0 ){
                    if(camera_.cloud(colorNormalCloud))
                        opipes_["cloud"]->flush(colorNormalCloud.makeShared());
                }
            }         
        }
}