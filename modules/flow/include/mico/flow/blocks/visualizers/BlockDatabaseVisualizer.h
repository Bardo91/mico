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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKDATABASEVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKDATABASEVISUALIZER_H_

#include <flow/Block.h>

#include <mutex>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkNamedColors.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCommand.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mico/flow/blocks/visualizers/VtkVisualizer3D.h>
#include <mico/base/map3d/Dataframe.h>

#ifdef HAS_DARKNET
    #include <mico/dnn/map3d/Entity.h>
#endif

#include <map>

namespace mico{
    class BlockDatabaseVisualizer: public flow::Block {
    public:
        virtual std::string name() override {return "Database Visualizer";}

        BlockDatabaseVisualizer();
        ~BlockDatabaseVisualizer();

        bool configure(std::unordered_map<std::string, std::string> _params) override{
            std::istringstream istr(_params["cs_scale"]);
            istr >> scaleCs_;
            return true;
        }
        std::vector<std::string> parameters() override { return {"cs_scale"}; }


        virtual QWidget * customWidget() override;

    private:
        int updateRender(int _id, const  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud, const  Eigen::Matrix4f &_pose);
        void updateCoordinates(Eigen::Matrix4f &_pose);

        void convertToVtkMatrix( const Eigen::Matrix4f &_eigMat, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

    private:
    
        std::unordered_map<int, std::shared_ptr<Dataframe<pcl::PointXYZRGBNormal>>> dataframes_;
    #ifdef HAS_DARKNET
        std::unordered_map<int, std::shared_ptr<Entity<pcl::PointXYZRGBNormal>>> entities_;
    #endif

        VtkVisualizer3D *visualizer_ = nullptr;

        std::map<int, vtkSmartPointer<vtkActor>>  actors_;
        std::vector<vtkSmartPointer<vtkActor>>  actorsToDelete_;
        std::vector<int> idsToDraw_;

        vtkSmartPointer<vtkActor> actorCs_;
        vtkSmartPointer<vtkActor> prevActorCs_ = nullptr;

        std::mutex actorsGuard_;
        float scaleCs_ = 1.0;
        bool idle_ = true;
        
        bool running_ = true;
        
    };

}

#endif