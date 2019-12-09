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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKTRAYECTORYVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_BLOCKTRAYECTORYVISUALIZER_H_

#include <mico/flow/blocks/visualizers/VtkVisualizer3D.h>

namespace mico{

    VtkVisualizer3D::VtkVisualizer3D(std::string _winName){
        // Setup render window, renderer, and interactor
        renderWindow->SetWindowName(_winName.c_str());
        renderWindow->AddRenderer(renderer);
        renderWindowInteractor->SetRenderWindow(renderWindow);
        spinOnceCallback_ = vtkSmartPointer<SpinOnceCallback>::New();
        spinOnceCallback_->interactor_ = renderWindowInteractor;
        renderer->SetBackground(vtkSmartPointer<vtkNamedColors>::New()->GetColor3d("Gray").GetData());

        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        widgetCoordinates_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        widgetCoordinates_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
        widgetCoordinates_->SetOrientationMarker( axes );
        widgetCoordinates_->SetInteractor( renderWindowInteractor );
        widgetCoordinates_->SetViewport( 0.0, 0.0, 0.4, 0.4 );
        widgetCoordinates_->SetEnabled( 1 );
        // widgetCoordinates_->InteractiveOn();


        // Visualize
        runInteractor_ = true;
        interactorThread_ = std::thread([&](){
            renderWindowInteractor->Initialize();
            while(runInteractor_){
                runLock_.lock();
                for(auto &fn: runOnUiThreadCalls_){
                    fn();
                }
                runLock_.unlock();

                renderWindowInteractor->Render();
                renderWindowInteractor->AddObserver(SpinOnceCallback::TimerEvent, spinOnceCallback_);
                auto timerId = renderWindowInteractor->CreateRepeatingTimer (10);    
                renderWindowInteractor->Start();
                renderWindowInteractor->DestroyTimer(timerId);
            }
            
        });
    }
    
    VtkVisualizer3D::~VtkVisualizer3D(){
        runInteractor_ = false;
        renderWindow->Finalize();
        renderWindowInteractor->GetRenderWindow()->Finalize();
        renderWindowInteractor->ExitCallback();
        renderWindowInteractor->TerminateApp();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(interactorThread_.joinable())
            interactorThread_.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

#endif