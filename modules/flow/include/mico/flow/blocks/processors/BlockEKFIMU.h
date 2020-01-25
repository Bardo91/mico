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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKEKFIMU_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKEKFIMU_H_

#include <flow/Block.h>

#include <mico/base/state_filtering/EKFImu.h>
#include <mico/base/cjson/json.h>
#include <Eigen/Eigen>
#include <fstream>

namespace mico{

    class BlockEKFIMU: public flow::Block{
    public:
        virtual std::string name() override {return "Inertial EKF";}

        BlockEKFIMU();
        // ~BlockEKFIMU(){};

        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

    private:
        bool startFilter_ = false;
        bool idle_ = true;
        EKFImu ekf_;
        Eigen::Vector3f gravity_ = {0.187647, 1.07087, -9.74372}; // CALIBRATION
        Eigen::Vector3f lastPosition_     = {0 , 0 , 0};
        Eigen::Vector3f lastAcceleration_ = {0 , 0 , 0};
		std::chrono::time_point<std::chrono::system_clock> prevT_;

        bool newObservation(Eigen::Matrix<double,6,1> _Zk);
    };

}

#endif