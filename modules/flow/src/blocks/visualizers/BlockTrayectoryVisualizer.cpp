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



#include <mico/flow/blocks/visualizers/BlockTrayectoryVisualizer.h>

#include <flow/Policy.h>
#include <flow/Outpipe.h>

#include <Eigen/Eigen>

#include <vtkInteractorStyleFlight.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>


namespace mico{

    BlockTrayectoryVisualizer::BlockTrayectoryVisualizer(): VtkVisualizer3D("Trajectory Visualizer") {
        // Init trajectory

        polyData->Allocate();
        polyData->SetPoints(points);
        mapper->SetInputData(polyData);
        actor->SetMapper(mapper);
        renderer->AddActor(actor);

        createPolicy({{"Camera Pose", "pose"}});

        registerCallback({"Camera Pose"}, 
                                [&](flow::DataFlow  _data){
                                    if(idle_){
                                        idle_ = false;
                                        Eigen::Matrix4f pose = _data.get<Eigen::Matrix4f>("Camera Pose");
                                        if(isFirstPoint){
                                            points->InsertNextPoint(pose(0,3), pose(1,3), pose(2,3));
                                            isFirstPoint = false;
                                        }else{
                                            vtkIdType connectivity[2];
                                            connectivity[0] = currentIdx_;
                                            connectivity[1] = currentIdx_+1;
                                            points->InsertNextPoint(pose(0,3), pose(1,3), pose(2,3));
                                            polyData->InsertNextCell(VTK_LINE,2,connectivity);
                                            currentIdx_++;

                                            polyData->Modified();
                                            

                                        }
                                        idle_ = true;
                                    }

                                }
                            );
    }

    BlockTrayectoryVisualizer::~BlockTrayectoryVisualizer(){
        // The interactor is handle by the parent class    
    }

}
