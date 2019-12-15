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

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>


namespace mico{
    #ifdef MICO_HAS_PANGOLIN
        int BlockVisualizerPangolin::sWinId = 0;

        BlockVisualizerPangolin::BlockVisualizerPangolin(){

            windowName_ = "pangolin_"+std::to_string(sWinId);
            sWinId++;

        
            renderThread_ = std::thread(&BlockVisualizerPangolin::renderCallback, this);    

        }
        
        BlockVisualizerPangolin::~BlockVisualizerPangolin(){

        }


        void BlockVisualizerPangolin::drawOnRenderThread(std::function<void()> &_fn){
            renderGuard_.lock();
            pendingDrawing_.push_back(_fn);
            renderGuard_.unlock();
        }

        void BlockVisualizerPangolin::renderCallback(){
            pangolin::CreateWindowAndBind(windowName_,640,480);
            glEnable(GL_DEPTH_TEST);
        
            pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
                pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
            );

            pangolin::Renderable tree;
            tree.Add( std::make_shared<pangolin::Axis>() );

            // Create Interactive View in window
            pangolin::SceneHandler handler(tree, s_cam);
            pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                    .SetHandler(&handler);

            d_cam.SetDrawFunction([&](pangolin::View& view){
                view.Activate(s_cam);
                tree.Render();
            });

            while( !pangolin::ShouldQuit() ) {
                renderGuard_.lock();
                for(auto &fn:pendingDrawing_){
                    fn();
                }
                renderGuard_.unlock();

                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                // Swap frames and Process Events
                pangolin::FinishFrame();
            }
        }
#endif
}

