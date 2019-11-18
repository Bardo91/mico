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



#include <mico/flow/blocks/visualizers/BlockDatabaseVisualizer.h>
#include <flow/Policy.h>
#include <flow/OutPipe.h>


#include <mico/base/map3d/Dataframe.h>

#include <Eigen/Eigen>
#include <vtkInteractorStyleFlight.h>
#include <vtkAxesActor.h>
#include <vtkAxes.h>
#include <vtkLine.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkLODActor.h>
#include <vtkFloatArray.h>
#include <vtkTubeFilter.h>
#include <vtkTransform.h>

#include <pcl/registration/transforms.h>

#include <pcl/filters/voxel_grid.h>

namespace mico{

    BlockDatabaseVisualizer::BlockDatabaseVisualizer(){
        // Setup render window, renderer, and interactor
        renderWindow->SetWindowName("Pointcloud Visualization");
        renderWindow->AddRenderer(renderer);
        renderWindowInteractor->SetRenderWindow(renderWindow);
        renderWindowInteractor->SetDesiredUpdateRate (30.0);
        spinOnceCallback_ = vtkSmartPointer<SpinOnceCallback>::New();
        spinOnceCallback_->interactor_ = renderWindowInteractor;
        renderWindowInteractor->AddObserver(SpinOnceCallback::TimerEvent, spinOnceCallback_);
    
        renderer->SetBackground(colors->GetColor3d("Gray").GetData());

        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        widgetCoordinates_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        widgetCoordinates_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
        widgetCoordinates_->SetOrientationMarker( axes );
        widgetCoordinates_->SetInteractor( renderWindowInteractor );
        widgetCoordinates_->SetViewport( 0.0, 0.0, 0.4, 0.4 );
        widgetCoordinates_->SetEnabled( 1 );

        // Visualize
        interactorThread_ = std::thread([&](){
            renderWindowInteractor->Initialize();
            vtkSmartPointer<vtkActor> prevActorCs = nullptr;
            while(running_){    //666 better condition for proper finalization.
                actorsGuard_.lock();
                // Update CF Pointclouds
                if(actorsToDelete_.size()>0){
                    for(auto &actor: actorsToDelete_)
                        renderer->RemoveActor(actor);
                }
                actorsToDelete_.clear();

                if(idsToDraw_.size() > 0){
                    for(auto id: idsToDraw_){
                        renderer->AddActor(actors_[id]);
                    }
                }

                // Update Pose
                if(actorCs_ && actorCs_ != prevActorCs){
                    if(prevActorCs){
                        renderer->RemoveActor(prevActorCs);
                    }
                    prevActorCs = actorCs_;
                    renderer->AddActor(actorCs_);
                }
                actorsGuard_.unlock();

                renderWindowInteractor->Render();
                auto timerId = renderWindowInteractor->CreateRepeatingTimer (10);
                
                renderWindowInteractor->Start();
                renderWindowInteractor->DestroyTimer(timerId);
            }
        });
        #ifdef HAS_DARKNET
        iPolicy_ = new flow::Policy({"dataframe", "pose", "v_entity"});
        #else
        iPolicy_ = new flow::Policy({"dataframe", "pose"});
        #endif

        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                        Dataframe<pcl::PointXYZRGBNormal>::Ptr df = std::any_cast<Dataframe<pcl::PointXYZRGBNormal>::Ptr>(_data["dataframe"]); 
                                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
                                        updateRender(df->id(), df->cloud(), df->pose());
                                        dataframes_[df->id()] = df;
                                }
                            );
        
        iPolicy_->registerCallback({"pose"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]);
                                        updateCoordinates(pose);
                                        idle_ = true;
                                    }
                                }
                            );

        #ifdef HAS_DARKNET
        iPolicy_->registerCallback({"v_entity"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        printf("New v_entity");
                                        std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities = std::any_cast<std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>>>(_data["v_entity"]); 
                                        for(auto &e: entities){
                                            auto dfs = e->dfs();
                                            updateRender(e->id(), e->cloud(dfs[0]), e->pose(dfs[0]));
                                            entities_[e->id()] = e;
                                        }
                                    }
                                }
                            );
        #endif

        redrawerThread_ = std::thread([&](){
            while(running_){    //666 better condition for proper finalization.
                for(auto &df: dataframes_){
                    if(df.second != nullptr && df.second->isOptimized()){
                        
                        actorsGuard_.lock();
                        actorsToDelete_.push_back(actors_[df.first]);
                        actorsGuard_.unlock();
                        updateRender(df.first, df.second->cloud(), df.second->pose());

                        df.second->isOptimized(false);
                    }
                }

                #ifdef HAS_DARKNET
                for(auto &e: entities_){
                    if(e.second != nullptr){
                        actorsGuard_.lock();
                        actorsToDelete_.push_back(actors_[e.first]);
                        actorsGuard_.unlock();
                        auto dfs = e.second->dfs();
                        updateRender(e.first, e.second->cloud(dfs[0]), e.second->pose(dfs[0]));
                    }
                }
                #endif

                std::this_thread::sleep_for(std::chrono::milliseconds(500));    // low frame rate.
            }

        });
    }

    BlockDatabaseVisualizer::~BlockDatabaseVisualizer(){
        running_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(interactorThread_.joinable())
            interactorThread_.join();
        
        if(redrawerThread_.joinable())
            redrawerThread_.join();


        renderWindowInteractor->GetRenderWindow()->Finalize();
        renderWindowInteractor->ExitCallback();
        renderWindowInteractor->TerminateApp();
    }

    void BlockDatabaseVisualizer::updateRender(int _id, const  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, const Eigen::Matrix4f &_pose){
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName ("Colors");

        pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxeler;
        voxeler.setLeafSize (0.1,0.1,0.1);
        pcl::PointCloud<pcl::PointXYZRGBNormal> cloudDrawn;
        voxeler.setInputCloud (_cloud);
        voxeler.filter (cloudDrawn);

        for(auto &p: cloudDrawn){
            points->InsertNextPoint (p.x, p.y, p.z);
            colors->InsertNextTuple3(p.r, p.g, p.b);
        }

        vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
        pointsPolydata->SetPoints(points);
        vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexFilter->SetInputData(pointsPolydata);
        vertexFilter->Update();
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->ShallowCopy(vertexFilter->GetOutput());
        polydata->GetPointData()->SetScalars(colors);

        // Visualization
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(2);

        vtkSmartPointer<vtkMatrix4x4> transformMat = vtkSmartPointer<vtkMatrix4x4>::New();
        convertToVtkMatrix(_pose, transformMat);
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        transform->SetMatrix(transformMat);
        transform->PostMultiply(); //this is the key line
        actor->SetUserTransform(transform);
        actor->Modified();
        
        actorsGuard_.lock();
        actors_[_id] = actor;
        idsToDraw_.push_back(_id);
        actorsGuard_.unlock();
    }

    // Code from pclvisualizer
    int feq (double a, double b) {
        return std::abs (a - b) < 1e-9;
    }

    // Code from pclvisualizer
    void quat_to_angle_axis (const Eigen::Quaternionf &qx, double &theta, double axis[3]) {
        double q[4];
        q[0] = qx.w();
        q[1] = qx.x();
        q[2] = qx.y();
        q[3] = qx.z();

        double halftheta = std::acos (q[0]);
        theta = halftheta * 2;
        double sinhalftheta = sin (halftheta);
        if (feq (halftheta, 0)) {
            axis[0] = 0;
            axis[1] = 0;
            axis[2] = 1;
            theta = 0;
        } else {
            axis[0] = q[1] / sinhalftheta;
            axis[1] = q[2] / sinhalftheta;
            axis[2] = q[3] / sinhalftheta;
        }
    }


    void BlockDatabaseVisualizer::updateCoordinates(Eigen::Matrix4f &_pose){
        vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
        axes->SetOrigin (_pose(0,3), _pose(1,3), _pose(2,3));
        axes->SetScaleFactor (scaleCs_);
        axes->Update ();

        vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
        axes_colors->Allocate (6);
        axes_colors->InsertNextValue (0.0);
        axes_colors->InsertNextValue (0.0);
        axes_colors->InsertNextValue (0.5);
        axes_colors->InsertNextValue (0.5);
        axes_colors->InsertNextValue (1.0);
        axes_colors->InsertNextValue (1.0);

        vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
        axes_data->GetPointData ()->SetScalars (axes_colors);

        vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
        axes_tubes->SetInputData (axes_data);
        axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
        axes_tubes->SetNumberOfSides (6);

        vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
        axes_mapper->SetScalarModeToUsePointData ();
        axes_mapper->SetInputConnection (axes_tubes->GetOutputPort ());

        vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
        axes_actor->SetMapper (axes_mapper);

        axes_actor->SetPosition (_pose(0, 3), _pose(1, 3), _pose(2, 3));

        Eigen::Matrix3f m;
        m =_pose.block<3,3>(0,0);
        Eigen::Quaternionf rf;
        rf = Eigen::Quaternionf(m);
        double r_angle;
        double r_axis[3];
        quat_to_angle_axis(rf,r_angle,r_axis);
        //
        axes_actor->SetOrientation(0,0,0);
        axes_actor->RotateWXYZ(r_angle*180/M_PI,r_axis[0],r_axis[1],r_axis[2]);
        
        actorsGuard_.lock();
        actorCs_ = axes_actor;
        actorsGuard_.unlock();
    }


    void BlockDatabaseVisualizer::convertToVtkMatrix( const Eigen::Matrix4f &_eigMat, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix){
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                vtk_matrix->SetElement (i, k, _eigMat(i, k));
    }
}
