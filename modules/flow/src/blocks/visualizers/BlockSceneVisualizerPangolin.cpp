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

                                        auto cube = e->boundingCube(firstDf);  // 0->xmax 1->xmin 2->ymax 3>ymin 4->zmax 5->zmin
                                        Eigen::Vector4f v1(cube[0], cube[3], cube[5], 0);
                                        Eigen::Vector4f v2(cube[0], cube[3], cube[4], 0);
                                        Eigen::Vector4f v3(cube[0], cube[2], cube[5], 0);
                                        Eigen::Vector4f v4(cube[0], cube[2], cube[4], 0);
                                        Eigen::Vector4f v5(cube[1], cube[3], cube[5], 0);
                                        Eigen::Vector4f v6(cube[1], cube[3], cube[4], 0);
                                        Eigen::Vector4f v7(cube[1], cube[2], cube[5], 0);
                                        Eigen::Vector4f v8(cube[1], cube[2], cube[4], 0);
                                        Eigen::Matrix4f ePose = e->pose(firstDf);
                                        Eigen::Matrix4f dfPose = e->dfpose(firstDf);
                                        ePose = dfPose * ePose;
                                        v1 = ePose * v1;
                                        v2 = ePose * v2;
                                        v3 = ePose * v3;
                                        v4 = ePose * v4;
                                        v5 = ePose * v5;
                                        v6 = ePose * v6;
                                        v7 = ePose * v7;
                                        v8 = ePose * v8;
                                        // draw cube
                                        // up face
                                        visualizer_->addLine({v1(0),v1(1),v1(2)}, {v2(0),v2(1),v2(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v2(0),v2(1),v2(2)}, {v4(0),v4(1),v4(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v4(0),v4(1),v4(2)}, {v3(0),v3(1),v3(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v3(0),v3(1),v3(2)}, {v1(0),v1(1),v1(2)}, {1,0,0,0.6});
                                        // down face
                                        visualizer_->addLine({v5(0),v5(1),v5(2)}, {v6(0),v6(1),v6(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v6(0),v6(1),v6(2)}, {v8(0),v8(1),v8(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v8(0),v8(1),v8(2)}, {v7(0),v7(1),v7(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v7(0),v7(1),v7(2)}, {v5(0),v5(1),v5(2)}, {1,0,0,0.6});
                                        // the other lines
                                        visualizer_->addLine({v1(0),v1(1),v1(2)}, {v5(0),v5(1),v5(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v3(0),v3(1),v3(2)}, {v2(0),v2(1),v2(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v2(0),v2(1),v2(2)}, {v6(0),v6(1),v6(2)}, {1,0,0,0.6});
                                        visualizer_->addLine({v4(0),v4(1),v4(2)}, {v8(0),v8(1),v8(2)}, {1,0,0,0.6});
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

