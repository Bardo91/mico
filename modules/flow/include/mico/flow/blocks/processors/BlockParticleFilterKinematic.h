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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKPARTICLEFILTERKINEMATIC_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKPARTICLEFILTERKINEMATIC_H_

#include <flow/Block.h>
#include <mico/base/state_filtering/ParticleFilterCPU.h>
#include <mico/base/map3d/Dataframe.h>
#include <Eigen/Eigen>
#include<bits/stdc++.h>


namespace mico{
    struct ObservationData{
        Eigen::Vector3f position;
    };

    class ParticleRobot : public ParticleInterface<ObservationData>{
    public:
        int sign(float _x);
        ParticleRobot();
        void simulate();
        void simulateReal();

        ObservationData observation();
        double computeWeight(ObservationData &_observation);
        Eigen::Vector3f position();

    private:
        Eigen::Vector3f mPosition;	// x, y, z
        Eigen::Vector3f mSpeed;	// x, y, z

    };

    inline double gauss(const double & _nu, const double & _sigma);
    Eigen::Vector3f mediumState(std::vector<ParticleRobot> _particles);


    class BlockParticleFilterKinematic: public flow::Block{
    public:
        static std::string name() {return "Particle Filter Kinematic";}

        BlockParticleFilterKinematic();
        // ~BlockOdometryRGBD(){};

        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

    private:
        bool idle_ = true;
        ParticleFilterCPU<ParticleRobot, ObservationData> *filter_;	

        double positionNoise_ = 0.5;
        double speedNoise_ = 0.1;
        double nParticles_ = 500;
    };

}

#endif