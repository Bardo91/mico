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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_VTKVISUALIZER3D_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_VTKVISUALIZER3D_H_

#include <flow/Block.h>

#include <mutex>
#include <functional>

#include <vtkDoubleArray.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkCommand.h>

namespace mico{

    /// Callback that stops the interactor to allow to perform other actions. 
    class SpinOnceCallback : public vtkCommand {
    public:
        static SpinOnceCallback *New() {
            SpinOnceCallback *cb = new SpinOnceCallback();
            return cb;
        }


        virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData)) {
            if (interactor_)
                interactor_->TerminateApp ();
        }
    public:
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
    };

    /// Base class that handles the basic creation of VTK interfaces
    class VtkVisualizer3D{
    public:
        VtkVisualizer3D(std::string _winName);
        ~VtkVisualizer3D();

        void runOnUiThread(std::function<void()> _fn){
            runLock_.lock();
            runOnUiThreadCalls_.push_back(_fn);
            runLock_.unlock();
        }

        void addActor(vtkSmartPointer<vtkActor> &_actor){
            renderer->AddActor(_actor);
        }

        void removeActor(vtkSmartPointer<vtkActor> &_actor){
            renderer->RemoveActor(_actor);
        }

    protected:
        // Setup render window, renderer, and interactor
        vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
        vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtkSmartPointer<vtkOrientationMarkerWidget> widgetCoordinates_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        
        vtkSmartPointer<SpinOnceCallback> spinOnceCallback_;
        
        std::vector<std::function<void()>> runOnUiThreadCalls_;
        std::mutex runLock_;

        bool runInteractor_ = false; 
        std::thread interactorThread_;
    };

}

#endif