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

#include <flow/flow.h>
// #include <gperftools/profiler.h>

#include <mico/flow/mico_flow.h>
#include <csignal>
#include <iostream>

using namespace flow;
using namespace mico;


class IntStreamerBlock: public Block{
public:
    static std::string name() {return "Int streamer";}
        IntStreamerBlock(){
            createPipe("time", "int");
        }

        virtual void loopCallback() override{
            auto t0 = std::chrono::high_resolution_clock::now();
            while(isRunningLoop()){
                auto t1 = std::chrono::high_resolution_clock::now();
                float diff = counter_ + std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0f;
                getPipe("time")->flush(int(diff));
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }      
        }

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override { 
            if(_params["init_value"] != ""){
                std::istringstream istr(_params["init_value"]);
                istr >> counter_;
                return true;
            }else{
                return false;
            } 
        };
        virtual std::vector<std::string> parameters() override{ return {"init_value"}; };
private:
    int counter_  = 1;
};


class IntCouterBlock: public Block{
public:
    static std::string name() {return "Int Couter";}
    IntCouterBlock(){
        createPolicy({{{"clock", "int"}}});
        registerCallback({"clock"}, 
                            [&](DataFlow _data){
                                int data = _data.get<int>("clock");
                                std::cout << data << std::endl;
                            }
                            );
    }
    ~IntCouterBlock(){ /*Nothing spetial*/}
};



using QtNodes::DataModelRegistry;

FlowVisualInterface manager;

void registerDataModels(FlowVisualInterface::RegistryType_ &_register) {

        _register->registerModel<FlowVisualBlock<IntStreamerBlock, true>>("0streamers");
        _register->registerModel<FlowVisualBlock<IntCouterBlock>>("0visualizer");
    // Casters
    _register->registerModel<FlowVisualBlock<BlockDataframeToPose>>         ("Cast");
    // _register->registerModel<FlowVisualBlock<BlockDataframeToCloud>>        ("Cast");
    // _register->registerModel<FlowVisualBlock<PoseDemux>>                    ("Cast");

    // Streamers
    _register->registerModel<FlowVisualBlock<StreamDataset, true>>          ("Streamers");
    _register->registerModel<FlowVisualBlock<StreamRealSense, true>>        ("Streamers");
    _register->registerModel<FlowVisualBlock<StreamPixhawk, true>>          ("Streamers");

    #ifdef MICO_USE_ROS
        // ROS Streamers
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberImu>>             ("ROS Streamers");
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberGPS>>             ("ROS Streamers");
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberCloud>>           ("ROS Streamers");
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberImage>>           ("ROS Streamers");
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberPoseStamped>>     ("ROS Streamers");
        _register->registerModel<FlowVisualBlock<BlockROSSubscriberEventArray>>      ("ROS Streamers");

        // ROS Publishers
        _register->registerModel<FlowVisualBlock<BlockROSPublisherPoseStamped>>      ("ROS Publishers");
        _register->registerModel<FlowVisualBlock<BlockROSPublisherPointCloud>>       ("ROS Publishers");
    #endif

    // DNN
    #ifdef HAS_DARKNET
        _register->registerModel<FlowVisualBlock<BlockDarknet>>             ("Detector");
    #endif    
    // SLAM
    _register->registerModel<FlowVisualBlock<BlockOdometryRGBD>>            ("SLAM");
    _register->registerModel<FlowVisualBlock<BlockOdometryPhotogrammetry>>  ("SLAM");
    _register->registerModel<FlowVisualBlock<BlockDatabaseMarkI>>           ("SLAM");
    _register->registerModel<FlowVisualBlock<BlockLoopClosure>>             ("SLAM");
    _register->registerModel<FlowVisualBlock<BlockOptimizerCF>>             ("SLAM");
    
    // Visualizers
    _register->registerModel<FlowVisualBlock<BlockImageVisualizer>>         ("Visualizers");
    _register->registerModel<FlowVisualBlock<BlockTrayectoryVisualizer>>    ("Visualizers");
    _register->registerModel<FlowVisualBlock<BlockPointCloudVisualizer>>    ("Visualizers");
    _register->registerModel<FlowVisualBlock<BlockDatabaseVisualizer>>      ("Visualizers");
    _register->registerModel<FlowVisualBlock<BlockSceneVisualizer>>         ("Visualizers");
    #ifdef MICO_HAS_PANGOLIN
        _register->registerModel<FlowVisualBlock<BlockVisualizerPangolin>>      ("Visualizers");
    #endif
    //Savers
    _register->registerModel<FlowVisualBlock<SaverImage>>                   ("Savers");
    _register->registerModel<FlowVisualBlock<SaverTrajectory>>              ("Savers");
    _register->registerModel<FlowVisualBlock<SaverEntity>>                  ("Savers");

    // Queuers
    _register->registerModel<FlowVisualBlock<BlockQueuer<QueuerTraitClusterframes>>> ("Queuer");
    _register->registerModel<FlowVisualBlock<BlockQueuer<QueuerTraitColor>>> ("Queuer");

    // State filtering
    _register->registerModel<FlowVisualBlock<BlockEKFIMU>>                  ("State Filtering");

    // Misc
    _register->registerModel<FlowVisualBlock<BlockPython>>                  ("Misc");

}


void signalHandler( int signum ) {
    // ProfilerStop();
    std::cout <<"signal"<< std::endl;
    manager.quit();
    exit(signum);  
}

int main(int _argc, char *_argv[]) {    
    signal(SIGINT, signalHandler);  

    #ifdef MICO_USE_ROS
        ros::init(_argc, _argv, "SLAM4KIDS");
        ros::AsyncSpinner spinner(4);
        spinner.start();
    #endif

    // Disable pcl warnings.
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // std::string profileName = "mico4kids_prob_"+std::to_string(time(NULL))+".log";
    // ProfilerStart(profileName.c_str());
    manager.setNodeRegisterFn(registerDataModels);
    manager.setCodeGeneratorCustoms({"mico/flow/mico_flow.h"},              // INCLUDEs
                                    {"mico REQUIRED"},                      // FINDS
                                    {"mico::mico-base", "mico::mico-flow"}  // LINKS
                                    );    
                                    
    return manager.init(_argc, _argv);;
}