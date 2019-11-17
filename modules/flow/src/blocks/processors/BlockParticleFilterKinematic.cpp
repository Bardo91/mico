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

#include <mico/flow/blocks/processors/BlockParticleFilterKinematic.h>
#include <flow/Policy.h>
#include <flow/OutPipe.h>

namespace mico{

    BlockParticleFilterKinematic::BlockParticleFilterKinematic(){
        iPolicy_ = new flow::Policy({"dataframe"});
        opipes_["dataframe"] = new flow::OutPipe("dataframe");
                                
        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                        if(idle_){
                                            idle_ = false;
                                            auto df = std::any_cast<std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>>(_data["dataframe"]);
                                                                                
                                            ObservationData obs;
                                            auto pose = df->pose();
                                            obs.position =pose.block<3,1>(0,3);
                                            filter_->step(obs);
                                            Eigen::Vector3f medState = mediumState(filter_->particles());

                                            // Update pose with filtered estimation.
                                            pose.block<3,1>(0,3) = medState;
                                            df->pose(pose);

                                            idle_ = true;
                                        }
                                    }
                                );

    }

    bool BlockParticleFilterKinematic::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first == "n_particles"){
                std::istringstream ss(_params["n_particles"]);
                ss >> nParticles_;
            } else if(param.first == "position_noise"){
                std::istringstream ss(_params["position_noise"]);
                ss >> positionNoise_;
            } else if(param.first == "speed_noise"){
                std::istringstream ss(_params["speed_noise"]);
                ss >> speedNoise_;
            
            }
        }

        filter_ = new ParticleFilterCPU<ParticleRobot, ObservationData>(nParticles_);
        filter_->init();

        return false;

    }
    
    std::vector<std::string> BlockParticleFilterKinematic::parameters(){
        return {"n_particles", "position_noise", "speed_noise"};
    }


    //-------------------------
    inline double gauss(const double & _nu, const double & _sigma) { 
        std::random_device rd; 
        std::mt19937 gen(rd()); 

        std::normal_distribution<> d(_nu, _sigma); 

        return d(gen); 
    } 

	int ParticleRobot::sign(float _x){
		return _x < 0? -1:1;
	}

	ParticleRobot::ParticleRobot() {
		mPosition[0] = gauss(0,0.3);
		mPosition[1] = gauss(0,0.3);
		mPosition[2] = gauss(0,0.3);
		mSpeed[0] = gauss(0,0.2);
		mSpeed[1] = gauss(0,0.2);
		mSpeed[2] = gauss(0,0.2);

		mSpeed[0] = fabs(mSpeed[0]) > 0.5? sign(mSpeed[0])*0.5 : mSpeed[0];
		mSpeed[1] = fabs(mSpeed[1]) > 0.5? sign(mSpeed[1])*0.5 : mSpeed[1];
		mSpeed[2] = fabs(mSpeed[2]) > 0.5? sign(mSpeed[2])*0.5 : mSpeed[2];
	}
	void ParticleRobot::simulate() { 
		mPosition[0] += mSpeed[0]*0.06;
		mPosition[1] += mSpeed[1]*0.06;
		mPosition[2] += mSpeed[2]*0.06;

		mSpeed[0] += gauss(0,0.05);
		mSpeed[1] += gauss(0,0.05);
		mSpeed[2] += gauss(0,0.05);

		mSpeed[0] = fabs(mSpeed[0]) > 1.0? sign(mSpeed[0])*1.0 : mSpeed[0];
		mSpeed[1] = fabs(mSpeed[1]) > 1.0? sign(mSpeed[1])*1.0 : mSpeed[1];
		mSpeed[2] = fabs(mSpeed[2]) > 1.0? sign(mSpeed[2])*1.0 : mSpeed[2];

	};

	void ParticleRobot::simulateReal() { 
		mPosition[0] += mSpeed[0]*0.06;
		mPosition[1] += mSpeed[1]*0.06;
		mPosition[2] += mSpeed[2]*0.06;
	};

	ObservationData ParticleRobot::observation(){
		ObservationData data;
		data.position = mPosition;
		return data;
	}

	double ParticleRobot::computeWeight(ObservationData &_observation) {  
		ObservationData fakeData = observation();
		ObservationData realData = _observation;

        double score = 1/(fakeData.position - realData.position).norm();

		return score;
	};

	Eigen::Vector3f ParticleRobot::position(){ 
        return mPosition; 
    }


    Eigen::Vector3f mediumState(std::vector<ParticleRobot> _particles){
        Eigen::Vector3f position = {0.0, 0.0, 0.0};
        for (unsigned i = 0; i < _particles.size(); i++){
            Eigen::Vector3f pos = _particles[i].position();
            position[0] += pos[0];
            position[1] += pos[1];
            position[2] += pos[2];
        }
        position[0] /= _particles.size();
        position[1] /= _particles.size();
        position[2] /= _particles.size();

        return position;
    }

}
