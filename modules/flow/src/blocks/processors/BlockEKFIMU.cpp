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

#include <mico/flow/blocks/processors/BlockEKFIMU.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>
// #include <mico/flow/streamers/StreamPose.h>
// #include <mico/flow/streamers/StreamDataframe.h>

namespace mico{

    BlockEKFIMU::BlockEKFIMU(){
        createPipe("pose", "mat44");

        createPolicy({{"Last Pose", "mat44"},{"Acceleration", "vec3"}});
        prevT_ = std::chrono::system_clock::now();
        registerCallback({"Last Pose"}, 
                                [&](flow::DataFlow _data){
                                    //startFilter_ = true;
                                    if(idle_){
                                        idle_ = false;
                                        Eigen::Matrix4f pose = _data.get<Eigen::Matrix4f>("Last Pose");
                                        startFilter_ = true;
                                        //if (pose.size == 0)
                                        Eigen::Vector3f position = pose.block<3,1>(0,3);
                                        lastPosition_ = position;
                                        // printf("position used in EKF: px %f,py %f,pz %f \n",position[0],position[1],position[2]);
                                        // New observation EKF
                                        Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
                                        Zk << double(position[0]) , double(position[1]) , double(position[2]), 
                                                                    double(lastAcceleration_[0]) , double(lastAcceleration_[1]) , double(lastAcceleration_[2]);
                                        if (!newObservation(Zk)){
                                            idle_ = true;
                                            return;
                                        }
                                        idle_ = true;
                                    }       
                                }
        );

        registerCallback({"Acceleration"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_ ){
                                        idle_ = false;
                                        if (!startFilter_){
                                            idle_ = true;
                                            return;
                                        }
                                        Eigen::Vector3f acc = _data.get<Eigen::Vector3f>("Acceleration");
                                        lastAcceleration_ = acc;
                                        // printf(" acceleration used in EKF: ax %f,ay %f,az %f \n",acc[0],acc[1],acc[2]);
                                        // New observation EKF
                                        Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
                                        Zk << lastPosition_[0] , lastPosition_[1] , lastPosition_[2] , 
                                              double(acc[0]) , double(acc[1]) , double(acc[2]);
                                        if (!newObservation(Zk)){
                                            idle_ = true;
                                            return;
                                        }
                                        idle_ = true;
                                    }   
                                
                                }
        );
    }

    bool BlockEKFIMU::configure(std::unordered_map<std::string, std::string> _params){
        std::string path;
        for(auto &param: _params){
            if(param.first =="EKFconfig"){
                path = param.second;
            }
        }
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cout << "Cannot open file." << std::endl;
            return false;
        }
        cjson::Json configFile;
        if (!configFile.parse(file)) {
            std::cout << "Cannot parse config file." << std::endl;
            return false;
        }
        if (configFile.contains("Extended_Kalman_Filter")) {
            if (!ekf_.init(configFile["Extended_Kalman_Filter"])) {
                printf("Error inicializating EKF parameters \n");
                return false;
            }
        }
        printf("EKF parameters inicializated \n");
        return true;
    }
    
    std::vector<std::string> BlockEKFIMU::parameters(){
        return {"EKFconfig"};
    }

    bool BlockEKFIMU::newObservation(Eigen::Matrix<double,6,1> _Zk){
        auto t1 = std::chrono::system_clock::now();
        auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT_).count()/1000.0f;
        // make new observation
        ekf_.stepEKF(_Zk,double(incT)); // double(0.3);
        prevT_ = t1;
        
        // get state vector
        Eigen::Matrix<double,12,1> Xk = ekf_.state();
        Eigen::Matrix4f poseEKF = Eigen::Matrix4f::Identity();
        Eigen::Vector3f xk;
        xk[0] = float(Xk(0,0));
        xk[1] = float(Xk(1,0));
        xk[2] = float(Xk(2,0));
        poseEKF.block<3,1>(0,3) = xk;
        getPipe("pose")->flush(poseEKF);
        return true;
    }
}
