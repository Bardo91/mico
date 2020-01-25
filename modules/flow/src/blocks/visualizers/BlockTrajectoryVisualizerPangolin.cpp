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

#include <mico/flow/blocks/visualizers/BlockTrajectoryVisualizerPangolin.h>

#include <QDialog>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>

namespace mico{
    #ifdef MICO_HAS_PANGOLIN

        class SpinWidgetWindow: public QDialog{
        public:
            SpinWidgetWindow(QWidget *parent = nullptr){
                layout_ = new QVBoxLayout();
                setLayout(layout_);
                
                layout_->addWidget(new QLabel("Select number of trajectories to be displayed."));

                spinBox_ = new QSpinBox();
                spinBox_->setMinimum(1);
                spinBox_->setMaximum(6);
                layout_->addWidget(spinBox_);
                connect(spinBox_, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _n){ this->n_ = _n; });

                setModal(true);
                setFocusPolicy(Qt::StrongFocus);
                setFocus();
                setWindowTitle("Trajectory selector");
            }

            int number() const{
                return n_;
            }

        private:
            int n_ = 1;
            QSpinBox * spinBox_;
            QVBoxLayout *layout_;
        };


        BlockTrajectoryVisualizerPangolin::BlockTrajectoryVisualizerPangolin(){
            
            SpinWidgetWindow nTrajSelector;
            nTrajSelector.exec();

            nTrajs_ = nTrajSelector.number();

            lastPositions_.resize(nTrajs_);
            isFirst_.resize(nTrajs_, true);

            std::map<std::string, std::string> policyInputs;
            for(unsigned i = 0; i < nTrajs_; i++){
                policyInputs["Camera Pose " +std::to_string(i)] = "mat44";
            }

            createPolicy(policyInputs);
            for(unsigned i = 0; i < nTrajs_; i++){
                registerCallback({"Camera Pose " +std::to_string(i)}, 
                                        std::bind([&](flow::DataFlow  _data, int _id){
                                            this->poseCallback(_data, _id);
                                        }, 
                                        std::placeholders::_1, 
                                        i)
                                    );
            }
        }
        
        BlockTrajectoryVisualizerPangolin::~BlockTrajectoryVisualizerPangolin(){
            if(visualizer_){
                delete visualizer_;
            }
        }

        void BlockTrajectoryVisualizerPangolin::poseCallback(flow::DataFlow  _data, int _id){
            Eigen::Matrix4f pose = _data.get<Eigen::Matrix4f>("Camera Pose " + std::to_string(_id));
            if(isFirst_[_id]){
                lastPositions_[_id] = pose.block<3,1>(0,3);
                isFirst_[_id] = false;
            }else{
                Eigen::Vector3f currPosition = pose.block<3,1>(0,3);
                visualizer_->addLine(lastPositions_[_id], currPosition, colorLines_[_id]);
                lastPositions_[_id] = currPosition;
            }
        }


        QWidget * BlockTrajectoryVisualizerPangolin::customWidget() {
            QGroupBox * box = new QGroupBox;
            
            QHBoxLayout * layout = new QHBoxLayout;
            QPushButton *button = new QPushButton("Start Visualizer");
            layout->addWidget(button);
            
            box->setLayout(layout);

            QWidget::connect(button, &QPushButton::clicked, [this](){
                if(!visualizer_){
                    visualizer_ = new PangolinVisualizer();
                }
            });

            return box;
        }


    #endif
}

