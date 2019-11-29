
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

#ifndef MICO_KIDS_BLOCKS_CASTBLOCKS_H_
#define MICO_KIDS_BLOCKS_CASTBLOCKS_H_

#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/Block.h>

#include <iostream>

namespace mico{
    //-----------------------------------------------------------------------------------------------------------------
    // DATAFRAME CASTERS
    class BlockDataframeToSomething: public flow::Block{
    public:
        BlockDataframeToSomething(){

            iPolicy_ = new flow::Policy({{{"Dataframe", "dataframe"}}});

            iPolicy_->registerCallback({"Dataframe"}, 
                                    [&](flow::DataFlow _data){
                                            if(idle_){
                                                idle_ = false;
                                                    auto df = _data.get<mico::Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Dataframe");  
                                                    opipes_[tagToGet()]->flush(dataToget(df));
                                                idle_ = true;
                                            }
                                        }
                                    );
        }

        // ~BlockDataframeToSomething(){};
    protected:
        bool idle_ = true;
        virtual std::any dataToget(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> &_df) = 0;
        virtual std::string tagToGet() = 0;

    };

    //-----------------------------------------------------------------------------------------------------------------
    class BlockDataframeToPose: public BlockDataframeToSomething{
    public:
        static std::string name() {return "Dataframe -> Pose";}
        BlockDataframeToPose(){ opipes_["Pose"] = new flow::Outpipe("Pose","mat44"); }
        // ~BlockDataframeToPose(){};

    protected:
        virtual std::any dataToget(mico::Dataframe<pcl::PointXYZRGBNormal>::Ptr &_df)override{
            return _df->pose();
        };
        
        virtual std::string tagToGet() override {return "pose";};
    };

//     //-----------------------------------------------------------------------------------------------------------------
//     class BlockDataframeToCloud: public BlockDataframeToSomething{
//     public:
//         static std::string name() {return "Dataframe -> Cloud";}
//         BlockDataframeToCloud(){ ostreams_["cloud"] = new StreamCloud(); }
//     protected:
//         virtual std::unordered_map<std::string, std::any> dataToget(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> &_df)override{
//             std::unordered_map<std::string, std::any> data;
//             data["cloud"] = _df->cloud;
//             return data;
//         };
        
//         virtual std::string tagToGet() override {return "cloud";};
//     };

//     //-----------------------------------------------------------------------------------------------------------------
//     // Pose Demux
//     class PoseDemux: public flow::Block{
//     public:
//         static std::string name() {return "Pose Demux";}
//         PoseDemux(){
//             callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
//             if(idle_){
//                 idle_ = false;
//                     auto pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]);  
//                     std::unordered_map<std::string, std::any> data;
//                     data["position"] = (Eigen::Vector3f)  pose.block<3,1>(0,3);
//                     ostreams_["position"]->manualUpdate(data);
//                     Eigen::Quaternionf q;
//                     q.matrix() =  pose.block<3,3>(0,0);
//                     data["orientation"] = q;
//                     ostreams_["orientation"]->manualUpdate(data);
//                 idle_ = true;
//             }
//         };
//         ostreams_["position"] = new StreamPosition();
//         ostreams_["orientation"] = new StreamOrientation();

//         setPolicy(new flow::PolicyAllRequired());
//         iPolicy_->setupStream("pose");
//     }

//     protected:
//         bool idle_ = true;

//     };
}

#endif