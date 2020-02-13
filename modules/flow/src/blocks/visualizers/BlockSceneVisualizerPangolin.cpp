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

#include <mico/flow/blocks/visualizers/BlockSceneVisualizerPangolin.h>

#include <QDialog>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>

#include <mico/base/map3d/Dataframe.h>
#ifdef HAS_DARKNET
    #include <mico/dnn/map3d/Entity.h>
#endif

namespace mico{
    #ifdef MICO_HAS_PANGOLIN
        BlockSceneVisualizerPangolin::BlockSceneVisualizerPangolin(){
            createPolicy({{"pose", "mat44"},{"Dataframe", "dataframe"}, {"Cloud", "cloud"},{"Entities", "v_entity"}});
            registerCallback(   {"pose"}, 
                                [&](flow::DataFlow  _data){
                                    if(!visualizer_){
                                        visualizer_ = new PangolinVisualizer();
                                    }

                                    Eigen::Matrix4f pose = _data.get<Eigen::Matrix4f>("pose");
                                    visualizer_->currentPose(pose);
                                }
                                );

            registerCallback({ "Dataframe" }, 
                                [&](flow::DataFlow  _data){
                                    if(!visualizer_){
                                        visualizer_ = new PangolinVisualizer();
                                    }
                                    auto df = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Dataframe");
                                    
                                    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud; 
                                    pcl::transformPointCloudWithNormals(*df->cloud(), cloud, df->pose());
                                    visualizer_->addPointCloud(cloud.makeShared());
                                    
                                    // Draw covisibility
                                    auto cov = df->covisibility();
                                    for(auto &odf:cov){
                                        Eigen::Vector3f pose = df->pose().block<3,1>(0,3);
                                        Eigen::Vector3f oPose = odf->pose().block<3,1>(0,3);
                                        visualizer_->addLine(pose, oPose, {1,0,0,0.6});
                                    }
                                }
                            );
#ifdef HAS_DARKNET
            registerCallback({ "Entities" }, 
                                [&](flow::DataFlow  _data){
                                    if(!visualizer_){
                                        visualizer_ = new PangolinVisualizer();
                                    }
                                    auto entities = _data.get<std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>>>("Entities"); 
                                    for(auto &e: entities){
                                        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud; 
                                        int firstDf = e->dfs()[0];
                                        pcl::transformPointCloudWithNormals(*e->cloud(firstDf), cloud, e->dfpose(firstDf));
                                        visualizer_->addPointCloud(cloud.makeShared());
                                    }
                                }
                            );
#endif             
            registerCallback({ "Cloud" }, 
                                [&](flow::DataFlow  _data){
                                    if(!visualizer_){
                                        visualizer_ = new PangolinVisualizer();
                                    }
                                    auto cloud = _data.get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>("Cloud"); 
                                    visualizer_->addPointCloud(cloud);
                                }
                            );
        }
        
        BlockSceneVisualizerPangolin::~BlockSceneVisualizerPangolin(){
                if(visualizer_){
                   delete visualizer_;
                }
        }


        QWidget * BlockSceneVisualizerPangolin::customWidget() {
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

