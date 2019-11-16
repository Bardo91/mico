#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <mico/base/map3d/utils3d.h>
#include <mico/base/map3d/Word.h>
#include <pcl/io/pcd_io.h>

namespace mico {

    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::init(cjson::Json _configFile)//, DatabaseCF<PointType_> *_database)
    {
        if(_configFile.contains("enable") && (bool) _configFile["enable"]){
            // mDatabase  = _database;
            mViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
            mViewer->setBackgroundColor(0.1, 0.1, 0.1, 0);
            mViewer->addCoordinateSystem(0.05, "base", 0);
            mViewer->addCoordinateSystem(0.02, "current_pose", 0);
            mViewer->registerKeyboardCallback(&SceneVisualizer::keycallback, *this, (void *)&mViewer);
            mViewer->registerMouseCallback(&SceneVisualizer::mouseEventOccurred, *this, (void *)&mViewer);
            mViewer->registerPointPickingCallback(&SceneVisualizer::pointPickedCallback, *this, (void*)&mViewer);
            mViewer->setCameraPosition (1.59696, 0.285761, -3.40482, -0.084178, -0.989503, -0.117468);
            // if (mDatabase == nullptr) {
            //     //std::cout << "[SceneVisualizer] Failing getting database instance" << std::endl;
            // }
            if (_configFile.contains("DenseVisualization")) {
                mDenseVisualization = (bool) _configFile["DenseVisualization"];
            }
            if (_configFile.contains("use_octree")) {
                mUseOctree = (bool) _configFile["use_octree"];
                mOctreeDepth = _configFile["octree_depth"];
            }else{
            }

            if(_configFile.contains("voxel_size")){
                mUseVoxel = true;
                double mVoxelSize = (double) _configFile["voxel_size"];
                mVoxeler.setLeafSize (mVoxelSize,mVoxelSize,mVoxelSize);

            }

            mCovisibilityNodeColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            mCovisibilityNodeColors->SetNumberOfComponents(3);
            mCovisibilityNodeColors->SetName("Colors");
            mCovisibilityNodes = vtkSmartPointer<vtkPoints>::New();
            mCovisibilityGraph = vtkSmartPointer<vtkPolyData>::New();
            mCovisibilityGraph->Allocate();
            mViewer->addModelFromPolyData(mCovisibilityGraph, "covisibility_graph");
        }
        return true;
    }

    

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::close(){
        if(mViewer)
            mViewer->close();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline SceneVisualizer<PointType_>::~SceneVisualizer(){
        if(mViewer)
            mViewer->close();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::drawDataframe(std::shared_ptr<mico::Dataframe<PointType_>> &_df, bool _drawPoints){
        if(!mViewer){
            return;
        }

        if(mExistingDf.find(_df->id()) != mExistingDf.end()){
            mViewer->removeCoordinateSystem("df_cs_" + std::to_string(_df->id()));
            mViewer->removeText3D("df_text_" + std::to_string(_df->id()));
            if(mUseOctree)
                mViewer->removePointCloud("octree");
            else
                mViewer->removePointCloud("df_cloud_" + std::to_string(_df->id()));
        }

        Eigen::Matrix4f dfPose = _df->pose();
        mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(dfPose), "df_cs_" + std::to_string(_df->id()));
        
        pcl::PointXYZ position(dfPose(0, 3), dfPose(1, 3), dfPose(2, 3));
        mViewer->addText3D(std::to_string(_df->id()), position, 0.015, 1,0,0, "df_text_" + std::to_string(_df->id()));
        
        if(_drawPoints){
            mViewer->removePointCloud("df_words_" + std::to_string(_df->id()));
            if (!_df->words().empty()) {
                typename pcl::PointCloud<PointType_>::Ptr cloudDictionary = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
                for (auto &w : _df->words()) {
                    std::shared_ptr<Word<PointType_>> word = w.second;
                    cloudDictionary->push_back(word->asPclPoint());
                }
                // Draw dictionary cloud
                mViewer->addPointCloud<PointType_>(cloudDictionary, "df_words_" + std::to_string(_df->id()));  
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "df_words_" + std::to_string(_df->id()));
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "df_words_" + std::to_string(_df->id()));
            }
        }
        

        // Draw cloud
        if (_df->cloud() != nullptr){ 
            if(mUseOctree){
                pcl::PointCloud<PointType_> cloud;
                pcl::transformPointCloudWithNormals(*_df->cloud(), cloud, dfPose);
                mOctreeVis.setInputCloud (cloud.makeShared());
                mOctreeVis.addPointsFromInputCloud ();

                OctreeIterator treeIt;
                OctreeIterator treeItEnd = mOctreeVis.end();


                
                // int depth = mOctreeVis.getTreeDepth() / 1.25;

                pcl::PointCloud<PointType_> denseCloud;
                PointType_ pt;
                Eigen::Vector3f voxel_min, voxel_max;
                for (treeIt = mOctreeVis.begin(mOctreeDepth); treeIt!=treeItEnd; ++treeIt) {
                    mOctreeVis.getVoxelBounds(treeIt, voxel_min, voxel_max);

                    pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
                    pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
                    pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
                    
                    denseCloud.push_back(pt);
                }

                mViewer->addPointCloud<PointType_>(denseCloud.makeShared(), "octree");
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "octree");
            }
            else{
                pcl::PointCloud<PointType_> cloudDrawn;
                if(mUseVoxel){
                    mVoxeler.setInputCloud (_df->cloud());
                    mVoxeler.filter (cloudDrawn);
                    mViewer->addPointCloud<PointType_>(cloudDrawn.makeShared(), "df_cloud_" + std::to_string(_df->id()));
                }else{
                    mViewer->addPointCloud<PointType_>(_df->cloud(), "df_cloud_" + std::to_string(_df->id()));
                }
                mViewer->updatePointCloudPose("df_cloud_" + std::to_string(_df->id()), Eigen::Affine3f(dfPose));
            }
        }

        // Draw covisibility.
        // std::cout << "Drawing covisibility. Existing prevous "<< mExistingDf.size() << " nodes." << std::endl;
        Eigen::Vector3f origin = {position.x, position.y, position.z};
        if(mExistingDf.find(_df->id()) != mExistingDf.end()){
            // std::cout << "Updating existing covisibility" << std::endl;
            updateNodeCovisibility(_df->id(), origin);
            if(_df->covisibility().size() != unsigned(mNodeCovisibilityCheckSum[_df->id()])){
                // std::vector<int> newCov(_df->covisibility().begin()+ mNodeCovisibilityCheckSum[_df->id()], 
                //                         _df->covisibility().end());
                // addCovisibility(_df->id(), newCov);
                // mNodeCovisibilityCheckSum[_df->id()] = _df->covisibility().size();
                assert(false);; //what is this?....
            }
        }else{
            // std::cout << "Created new node" << std::endl;
            insertNodeCovisibility(origin);
            addCovisibility(_df->id(), _df->covisibility());
            mNodeCovisibilityCheckSum[_df->id()] = _df->covisibility().size();
            
            mCovisibilityGraph->SetPoints(mCovisibilityNodes);
            mCovisibilityGraph->GetPointData()->SetScalars(mCovisibilityNodeColors);
        }
        mExistingDf[_df->id()] = true;
        mDataframes[_df->id()] = _df; // 666 Duplicated.....

        mCovisibilityNodes->Modified();

        mViewer->spinOnce(10, true);
    }


    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::drawWords(std::map<int, std::shared_ptr<Word<PointType_>>> _words){
        mViewer->removePointCloud("words");
        if (!_words.empty()) {
            typename pcl::PointCloud<PointType_>::Ptr cloudDictionary = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
            for (auto &w : _words) {
                std::shared_ptr<Word<PointType_>> word = w.second;
                cloudDictionary->push_back(word->asPclPoint());
            }
            // Draw dictionary cloud
            mViewer->addPointCloud<PointType_>(cloudDictionary, "words");  
            mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "words");
            mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "words");
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::checkAndRedrawCf(){
        for(auto &cf: mDataframes){
            if(cf.second->isOptimized()){
                // std::cout << "CF: " << cf.first << " is optimized, redrawing" << std::endl;
                drawDataframe(cf.second);
                cf.second->isOptimized(false);
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::updateDataframe(int _dfId, const Eigen::Matrix4f &_newPose){
        if(!mViewer)
            return;
            
        if (mViewer->contains("df_cs_" + std::to_string(_dfId))) {
            mViewer->removeCoordinateSystem("df_cs_" + std::to_string(_dfId));
            mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_newPose), "df_cs_" + std::to_string(_dfId));

            mViewer->removeText3D("df_text_" + std::to_string(_dfId));
            pcl::PointXYZ position(_newPose(0, 3), _newPose(1, 3), _newPose(2, 3));
            mViewer->addText3D(std::to_string(_dfId), position, 0.015, 1,0,0, "text_" + std::to_string(_dfId));

            mViewer->updatePointCloudPose("df_cloud_" + std::to_string(_dfId), Eigen::Affine3f(_newPose));
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::draw3DMatches(pcl::PointCloud<PointType_> _pc1, pcl::PointCloud<PointType_> _pc2){
        if(!mViewer)
            return false;
            
        assert(_pc1.size() == _pc2.size());

        for(unsigned i = 0 ; i<_pc1.size(); i++){
            mViewer->addLine(_pc1[i], _pc2[i], "line_"+std::to_string(i));
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::updateCurrentPose(const Eigen::Matrix4f &_pose){
        if(!mViewer)
            return false;
            
        mViewer->removeCoordinateSystem("current_pose");
        mViewer->addCoordinateSystem(0.02, Eigen::Affine3f(_pose), "current_pose");
        mViewer->removeText3D("current_pose_text");
        pcl::PointXYZ position(_pose(0, 3), _pose(1, 3), _pose(2, 3));
        mViewer->addText3D("current_pose", position, 0.01, 1.0, 0.2, 0.2, "current_pose_text");
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::insertNodeCovisibility(const Eigen::Vector3f &_position){
        const unsigned char green[3] = {0, 255, 0};

        mCovisibilityNodes->InsertNextPoint(    _position[0], 
                                                _position[1], 
                                                _position[2]);

        mCovisibilityNodeColors->InsertNextTuple3(green[0], green[1], green[2]);
        
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::updateNodeCovisibility(int _id, const Eigen::Vector3f &_position){
        double point[3] = {_position[0],_position[1], _position[2]};
        mCovisibilityNodes->SetPoint(_id, point);
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::addCovisibility(int _id, const std::vector<typename Dataframe<PointType_>::Ptr> &_others){
        for(auto &other:_others){ 
            vtkIdType connectivity[2];
            connectivity[0] = _id;
            connectivity[1] = other->id();
            mCovisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity); //Connects the first and fourth point we inserted into a line
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::pause() {
        if(!mViewer)
            return;
            
        mPause=true;
        while (mPause) {
            mViewer->spinOnce(10, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cv::waitKey(10);
        }
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::spinOnce(){
        if(!mViewer)
            return;
            
        mViewer->spinOnce(10, true);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data) {
        for(auto &callback:mCustomCallbacks){
            callback(_event, _data);
        }

        if (_event.keyDown() && _event.getKeySym() == "z") {
            std::cout << "[Visualizer] Toogle pause" << std::endl;
            if(mPause==true) mPause = false;
            else pause();
        }
        else if (_event.keyDown() && _event.getKeySym() == "v") {
        // #ifdef USE_DBOW2
        //     // Init DBoW2 vocabulary
        //     const int K = 6; // From article Bags of Binary Words for Fast Place Recognition inImage Sequences
        //     const int L = 4;
        //     //const DBoW2::WeightingType weight = DBoW2::TF_IDF;  //TODO: Scoring type and weight type? 
        //     //const DBoW2::ScoringType score = DBoW2::L2_NORM;
        //     //OrbVocabulary voc(K, L, weight, score);
        //     OrbVocabulary voc(K, L);

        //     auto dfs = mDatabase->dataframes_;
        //     std::vector<std::vector<cv::Mat>> allFeatures;
        //     for (unsigned i = 0; i < dfs.size(); i++) {
        //         allFeatures.push_back(std::vector<cv::Mat>());
        //         for (int r = 0; r < dfs[i]->featureDescriptors.rows; r++) {
        //             allFeatures[i].push_back(dfs[i]->featureDescriptors.row(r));
        //         }
        //     }
        //     voc.create(allFeatures);
        //     voc.save("vocabulary_dbow2_livingroom_orb_k" + std::to_string(K) + "L" + std::to_string(L) + ".xml");
        //     std::cout << "Vocabulary saved in vocabulary_dbow2_livingroom_orb_k" + std::to_string(K) + "L" + std::to_string(L) + ".xml" << std::endl;
        // #else
        //     std::cout << "DBOW 2 not installed, or library not compiled with it. Cant compute dictionarty." << std::endl;
        // #endif
        }
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void){
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

      if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
          event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease){}
        
        
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void*viewer_void) {
        // event.getPoint(x,y,z);
        // for(auto &w: mDatabase->wordDictionary_){
        //     if(w.second->point[0]==x && w.second->point[1]==y && w.second->point[2]==z){
        //         std::cout << " Clicked word: " + std::to_string(w.second->id()) << std::endl;
        //     }
        // }
        // std::cout << "Point clicked at position (" << x << ", " << y << ", " << z << ")" << std::endl;  
    }; 

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::addCustomKeyCallback(CustomCallbackType _callback){
        mCustomCallbacks.push_back(_callback);
    }
} // namespace mico