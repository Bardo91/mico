//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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


namespace mico {

    template<typename PointType_>
    inline Entity<PointType_>::Entity(int _id, int _dataframeId, int _label, float _confidence, std::vector<float> _boundingbox){
        id_ = _id;
        label_ = _label;
        confidence_[_dataframeId] = _confidence;
        boundingbox_[_dataframeId] = _boundingbox;
        dfs_.push_back(_dataframeId);
    }

    template<typename PointType_>
    inline Entity<PointType_>::Entity(int _id, int _label, float _confidence, std::vector<float> _boundingbox):
    Entity(_id, 0,_label, _confidence,  _boundingbox){
    }

    template<typename PointType_>
    inline bool Entity<PointType_>::computePose(int _dataframeId){   // 666 _dataframeId to compute pose with several clouds        
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        std::vector<float> bc;
        // Compute principal directions
        if(!computePCA(*clouds_[_dataframeId], pose, bc))
            return false;

        poses_[_dataframeId] = pose;   // to global pose 666 check this
        // poses_[_dataframeId] = pose * covisibility_[_dataframeId];   // to global pose 666 check this
        positions_[_dataframeId] = pose.block(0,3,3,1);
        Eigen::Quaternionf q(pose.block<3,3>(0,0));
        orientations_[_dataframeId] = q;
        boundingcube_[_dataframeId] = bc;

        float width = bc[0] - bc[1]; 
        float heigth = bc[2] - bc[3]; 
        float deep = bc[4] - bc[5];
        auto globalCubePose = covisibility_[_dataframeId] * pose;
        cube_ = std::make_shared<Cube>(1, globalCubePose, width, heigth, deep);
        return true;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::updateCovisibility(int _dataframeId, Eigen::Matrix4f &_pose){
        covisibility_[_dataframeId] = _pose;

        // check for new dataframe
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    }
    
    template<typename PointType_>
    inline int Entity<PointType_>::id() const{
        return id_;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::pose(int _dataframeId, Eigen::Matrix4f &_pose){
            poses_[_dataframeId]   = _pose;
            positions_[_dataframeId]   = _pose.block<3,1>(0,3);
            orientations_[_dataframeId]   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Entity<PointType_>::pose(int _dataframeId){
        return poses_[_dataframeId];
    }

    template<typename PointType_>
    inline void Entity<PointType_>::dfpose(int _dataframeId, Eigen::Matrix4f &_pose){
            covisibility_[_dataframeId]   = _pose;
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Entity<PointType_>::dfpose(int _dataframeId){
        return covisibility_[_dataframeId];
    }

    template<typename PointType_>
    inline void Entity<PointType_>::projections(int _dataframeId, std::vector<cv::Point2f> _projections){
        projections_[_dataframeId] = _projections;
        // check for new dataframe
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    }

    template<typename PointType_>
    inline std::vector<cv::Point2f> Entity<PointType_>::projections(int _dataframeId){
        return projections_[_dataframeId];
    }

    template<typename PointType_>
    inline void Entity<PointType_>::descriptors(int _dataframeId, cv::Mat _descriptors){
        descriptors_[_dataframeId] = _descriptors.clone();
        // check for new dataframe
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    }

    template<typename PointType_>
    inline cv::Mat Entity<PointType_>::descriptors(int _dataframeId){
        return descriptors_[_dataframeId];
    }

    template<typename PointType_>
    inline void Entity<PointType_>::cloud(int _dataframeId, typename pcl::PointCloud<PointType_>::Ptr &_cloud){
        clouds_[_dataframeId] = _cloud;
        // check for new dataframe
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr  Entity<PointType_>::cloud(int _dataframeId){
        return clouds_[_dataframeId];
    }

    template<typename PointType_>
    inline void Entity<PointType_>::boundingbox(int _dataframeId, std::vector<float> _bb){
        boundingbox_[_dataframeId] = _bb;
        // check for new dataframe
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    }; 

    template<typename PointType_>
    inline std::vector<float> Entity<PointType_>::boundingbox(int _dataframeId){
        return boundingbox_[_dataframeId];
    };

    template<typename PointType_>    
    inline void Entity<PointType_>::boundingCube(int _dataframeId, std::vector<float> _bc){
        boundingcube_[_dataframeId] = _bc;
        if(std::find(dfs_.begin(), dfs_.end(), _dataframeId) == dfs_.end())
            dfs_.push_back(_dataframeId);
    };

    template<typename PointType_>    
    inline std::vector<float> Entity<PointType_>::boundingCube(int _dataframeId){
        return boundingcube_[_dataframeId];
    };

    template<typename PointType_>    
    inline std::vector<int> Entity<PointType_>::dfs(){
        return dfs_;
    };

    template<typename PointType_>    
    inline std::shared_ptr<Cube> Entity<PointType_>::cube(){
        return cube_;
    };

    template<typename PointType_>    
    inline float Entity<PointType_>::percentageOverlapped(std::shared_ptr<mico::Entity<PointType_>> _e){

        std::vector<Eigen::Vector3f> inter;
        auto queryCube = _e->cube();
        // compute intersection of two cubes
        queryCube->clipConvexPolyhedron(cube_, inter);
        // calculate volume overlapped
        float percentage = queryCube->computeVolumeFromPoints(inter) / queryCube->getVolume();
        float selfPercentage = cube_->computeVolumeFromPoints(inter) / cube_->getVolume();
        if(selfPercentage < percentage)
            return selfPercentage;
        return percentage;
    };

    template<typename PointType_>    
    inline void Entity<PointType_>::update(std::shared_ptr<mico::Entity<PointType_>> _e){
        for(auto &df: _e->dfs()){
            confidence_[df] = _e->confidence(df);
            boundingCube(df, boundingCube(df));
            dfs_.push_back(df);
        }
    }

    template<typename PointType_>    
    inline void Entity<PointType_>::confidence(int _df, float _confidence){
        if ( m.find(_df) == m.end() ) {
            // not found 
            confidence_[_df] = _confidence;
        } else {
            // found
            std::cerr << "[Entity] Trying to change the confidences of an already created object" << std::endl;
        }
    }

    template<typename PointType_>    
    inline float Entity<PointType_>::confidence(int _df){
        return confidence_[_df];
    }

}