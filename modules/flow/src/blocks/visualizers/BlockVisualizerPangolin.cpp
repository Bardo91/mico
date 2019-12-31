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

#include <mico/flow/blocks/visualizers/BlockVisualizerPangolin.h>

namespace mico{
    #ifdef MICO_HAS_PANGOLIN

        BlockVisualizerPangolin::BlockVisualizerPangolin(){
            
            createPolicy({{"Camera Pose", "mat44"}});
            registerCallback({"Camera Pose"}, 
                                    [&](flow::DataFlow  _data){
                                        if(idle_){
                                            idle_ = false;
                                            Eigen::Matrix4f pose = _data.get<Eigen::Matrix4f>("Camera Pose");
                                            if(isFirst_){
                                                lastPosition_ = pose.block<3,1>(0,3);
                                                isFirst_ = false;
                                            }else{
                                                Eigen::Vector3f currPosition = pose.block<3,1>(0,3);
                                                visualizer_.addLine(lastPosition_, currPosition);
                                                lastPosition_ = currPosition;
                                            }
                                            idle_ = true;
                                        }

                                    }
                                );
        }
        
        BlockVisualizerPangolin::~BlockVisualizerPangolin(){

        }

    #endif
}
