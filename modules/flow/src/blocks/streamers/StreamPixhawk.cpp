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


#include <mico/flow/blocks/streamers/StreamPixhawk.h>
#include <flow/Outpipe.h>

#include <Eigen/Eigen>

namespace mico{
        StreamPixhawk::StreamPixhawk(){
            opipes_["Acceleration"] = new flow::Outpipe("Acceleration", "vec3");
            opipes_["Orientation"] = new flow::Outpipe("Orientation", "vec4");
            opipes_["Angular Speed"] = new flow::Outpipe("Angular Speed", "vec3");
            opipes_["Position"] = new flow::Outpipe("Position", "vec3");
            opipes_["Pose"] = new flow::Outpipe("Pose", "mat44");
        }

        bool StreamPixhawk::configure(std::unordered_map<std::string, std::string> _params) {
            if(runLoop_) // Cant configure if already running.
                return false;

            #ifdef HAS_MAVSDK
                return px_.init(_params);
            #else
                std::cout << "Mico compiled without MAVSDK, cant use pixhawk streamer" << std::endl;
                return false;
            #endif
        }
        
        std::vector<std::string> StreamPixhawk::parameters(){
            return {
                "connection"
            };
        }

        void StreamPixhawk::loopCallback() {
            #ifdef HAS_MAVSDK
            while(runLoop_){
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 666 Configure it as px freq
                if(opipes_["Acceleration"]->registrations() !=0 ){
                    opipes_["Acceleration"]->flush(px_.acceleration());     
                }
                if(opipes_["Orientation"]->registrations() !=0 ){
                    opipes_["Orientation"]->flush(px_.orientation());
                }
                if(opipes_["Angular Speed"]->registrations() !=0 ){
                    opipes_["Angular Speed"]->flush(px_.angularSpeed());
                }
                if(opipes_["Position"]->registrations() !=0 ){
                    opipes_["Position"]->flush(px_.position());
                }
                if(opipes_["Pose"]->registrations() !=0 ){
                    auto position = px_.position();
                    auto orientation = px_.orientation();
                    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                    pose.block<3,1>(0,3) = position;
                    pose.block<3,3>(0,0) = orientation.matrix();
                    opipes_["Pose"]->flush(pose);
                }
            }      
            #else
                std::cout << "Mico compiled without MAVSDK, cant use pixhawk streamer" << std::endl;
            #endif
        }
}