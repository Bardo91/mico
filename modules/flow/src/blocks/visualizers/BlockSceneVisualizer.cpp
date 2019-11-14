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



#include <mico/flow/blocks/visualizers/BlockSceneVisualizer.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>


#include <mico/base/map3d/Dataframe.h>

#ifdef HAS_DARKNET
    #include <mico/dnn/map3d/Entity.h>
#endif

#include <Eigen/Eigen>


namespace mico{

    // Static member initialiation
    bool BlockSceneVisualizer::sAlreadyExisting_ = false;

    BlockSceneVisualizer::BlockSceneVisualizer(){
        if(sAlreadyExisting_)
            return;

        sAlreadyExisting_ = true;
        sBelonger_ = true;



        iPolicy_ = new Policy({"pose", "dataframe", "v_entity"});

        iPolicy_->registerCallback({"pose" }, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]); 
                                    poseGuard_.lock();
                                    lastPose_ = pose;
                                    poseGuard_.unlock();
                                    hasPose = true;
                                }
                            );

        iPolicy_->registerCallback({"dataframe" }, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    Dataframe<pcl::PointXYZRGBNormal>::Ptr df = std::any_cast<Dataframe<pcl::PointXYZRGBNormal>::Ptr>(_data["dataframe"]); 
                                    queueDfGuard_.lock();
                                    queueDfs_.push_back(df);
                                    queueDfGuard_.unlock();
                                    idle_ = true;
                                }
                            );
        iPolicy_->registerCallback({"v_entity" }, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities = std::any_cast<std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>>>(_data["v_entity"]); 
                                    queueEntitiesGuard_.lock();
                                    queueEntities_.push_back(entities);
                                    queueEntitiesGuard_.unlock();
                                    idle_ = true;
                                }
                            );
        
    }

    BlockSceneVisualizer::~BlockSceneVisualizer(){
        if(sBelonger_){
            run_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(spinnerThread_.joinable())
                spinnerThread_.join();
            sAlreadyExisting_ = false;
        }
    }


    bool BlockSceneVisualizer::configure(std::unordered_map<std::string, std::string> _params) {
        {
            std::istringstream istr(_params["voxel_size"]);
            istr >> voxelSize_;
        }
        {
            std::istringstream istr(_params["use_octree"]);
            istr >> useOctree;
        }
        {
            std::istringstream istr(_params["octree_depth"]);
            istr >> octreeDepth;
        }

        if(!hasBeenInitialized_){
            init();
        }

        return true;
    }

    std::vector<std::string> BlockSceneVisualizer::parameters() {
        return {"voxel_size", "use_octree", "octree_depth"};
    }

    void BlockSceneVisualizer::init(){
        spinnerThread_ = std::thread([this]{
            cjson::Json configFile;
            configFile["enable"] = true;
            if(voxelSize_ > 0)
                configFile["voxel_size"] = voxelSize_;
            if(useOctree){
                configFile["use_octree"] = useOctree;
                configFile["octree_depth"] = octreeDepth;
            }

            sceneVisualizer_.init(configFile);

            while(run_){
                if(hasPose){
                    // update last pose
                    poseGuard_.lock();
                    Eigen::Matrix4f pose = lastPose_;
                    poseGuard_.unlock();
                    sceneVisualizer_.updateCurrentPose(pose);
                }

                while(queueDfs_.size() > 0){
                    queueDfGuard_.lock();
                    auto cf = queueDfs_.front();
                    queueDfs_.pop_front();
                    queueDfGuard_.unlock();
                    sceneVisualizer_.drawDataframe(cf);
                }

                while(queueEntities_.size() > 0){
                    queueEntitiesGuard_.lock();
                    auto e = queueEntities_.front();
                    queueEntities_.pop_front();
                    queueEntitiesGuard_.unlock();
                    sceneVisualizer_.drawEntity(e, true, 0.4);
                }

                // Check optimizations.
                sceneVisualizer_.checkAndRedrawCf();
                
                // Spin once.
                sceneVisualizer_.spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Self destroy
            sceneVisualizer_.close();
        });
    }

}
