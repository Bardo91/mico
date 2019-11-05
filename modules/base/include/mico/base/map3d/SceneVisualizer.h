
#ifndef SLAMMARKI_SCENEVISUALIZER_H_
#define SLAMMARKI_SCENEVISUALIZER_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

#include <mico/base/map3d/Dataframe.h>
#include <mico/base/map3d/ClusterFrames.h>
#include <mico/base/map3d/DatabaseCF.h>
#include <mico/base/cjson/json.h>

#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <mico/base/utils/LogManager.h>

namespace mico {
    template <typename PointType_>
    class SceneVisualizer
    {
    public:
        SceneVisualizer():mOctreeVis(0.01){};
        ~SceneVisualizer();

        /// Initializes SceneVisualizer parameters
        bool init(cjson::Json _configFile);//, DatabaseCF<PointType_> *_database);

        void close();

        void cleanAll(){
            if(!mViewer)
                return;
                
            mViewer->removeAllPointClouds();
            mViewer->removeAllCoordinateSystems();
            mViewer->removeAllShapes();
        }



        void drawDataframe(std::shared_ptr<mico::Dataframe<PointType_>> &_kf);
        void updateDataframe(int _frameId, Eigen::Matrix4f &_newPose);
        void drawClusterframe(std::shared_ptr<mico::ClusterFrames<PointType_>> &_cluster, bool _drawPoints = false);
        void updateClusterframe(int _clusterId, Eigen::Matrix4f &_newPose);
        void drawWords(std::map<int, std::shared_ptr<Word<PointType_>>> _words);

        // Check if clusterframes have been optimized to updated them
        void checkAndRedrawCf();

        // Draw every word optimized
        bool draw3DMatches(pcl::PointCloud<PointType_> _pc1, pcl::PointCloud<PointType_> _pc2);
        
        // Draw every word optimized
        bool updateCurrentPose(Eigen::Matrix4f &_pose);

        void pause();
        void spinOnce();
        void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);
        void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);
        void pointPickedCallback(const pcl::visualization::PointPickingEvent &event,void*viewer_void);

        typedef std::function<void(const pcl::visualization::KeyboardEvent &, void *)> CustomCallbackType;
        void addCustomKeyCallback(CustomCallbackType _callback);

    private:
        void insertNodeCovisibility(Eigen::Vector3f &_position);
        void updateNodeCovisibility(int _id, Eigen::Vector3f &_position);
        void addCovisibility(int _id, std::vector<int> &_others);
    
    private:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;

        float x,y,z;
        typename pcl::octree::OctreePointCloudOccupancy<PointType_> mOctreeVis;
        typedef typename pcl::octree::OctreePointCloudOccupancy<PointType_>::Iterator OctreeIterator;

        bool mUseOctree = false;
        int mOctreeDepth = 1;

        std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> mClustersFrames;

        typename pcl::PointCloud<PointType_>::Ptr wordCloud = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
        
        bool mDenseVisualization=true;

        bool mPause = false;

        std::vector<CustomCallbackType> mCustomCallbacks;

        vtkSmartPointer<vtkPolyData> mCovisibilityGraph;
        vtkSmartPointer<vtkUnsignedCharArray> mCovisibilityNodeColors;
        vtkSmartPointer<vtkPoints> mCovisibilityNodes;

        std::map<int,bool> mExistingCluster;
        std::map<int,int> mNodeCovisibilityCheckSum;

        bool mUseVoxel = false;
        pcl::VoxelGrid<PointType_> mVoxeler;
        
    };
} // namespace mico

#include "SceneVisualizer.inl"

#endif // SLAMMARKI_VISUALIZER_H_