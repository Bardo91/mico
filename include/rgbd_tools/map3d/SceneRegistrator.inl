//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#include <thread>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>

#include <rgbd_tools/map3d/BundleAdjuster.h>

#include <rgbd_tools/map3d/utils3d.h>

namespace rgbd{
//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline SceneRegistrator<PointType_>::SceneRegistrator(){

}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::addDataframe(std::shared_ptr<DataFrame<PointType_>> &_kf){
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    if(mLastKeyframe != nullptr){
        if(_kf->featureCloud == nullptr && _kf->cloud== nullptr && _kf->left.rows != 0){
            std::vector<cv::DMatch> matches;
            matchDescriptors(_kf->featureDescriptors, mLastKeyframe->featureDescriptors, matches);

            std::vector<cv::Point2f> projsForPoseCurrent(matches.size()), projsForPosePrev(matches.size());
            for(unsigned i = 0; i < matches.size(); i++){
                projsForPoseCurrent[i] = _kf->featureProjections[matches[i].queryIdx];
                projsForPosePrev[i] = mLastKeyframe->featureProjections[matches[i].trainIdx];
            }

            if(matches.size() < 20){
                return false;
            }

            cv::Mat R, t, maskInliers;
            cv::Mat essential = cv::findEssentialMat(projsForPoseCurrent, projsForPosePrev,_kf->intrinsic,cv::RANSAC, 0.999, 1, maskInliers);
            cv::recoverPose(essential, projsForPoseCurrent, projsForPosePrev,_kf->intrinsic,R, t, maskInliers);

            for(unsigned i = 0 ; i < 3 ; i++){
                for(unsigned j = 0; j < 3; j++){
                    transformation(i,j) = R.at<double>(i,j);
                }
                transformation(i,3) = t.at<double>(i);
            }
            int numInliers = cv::sum(maskInliers)[0];
            if (numInliers >= 12) { // PARAMETRIZE INLIERS This process is somehow repeated in transformation between features! might be good to colapse!
                _kf->multimatchesInliersKfs[mLastKeyframe->id];
                mLastKeyframe->multimatchesInliersKfs[_kf->id];
                for(unsigned i = 0; i < maskInliers.rows; i++){
                    if(maskInliers.at<uchar>(i) == 1){
                        _kf->multimatchesInliersKfs[mLastKeyframe->id].push_back(matches[i]);
                        mLastKeyframe->multimatchesInliersKfs[_kf->id].push_back(cv::DMatch(matches[i].trainIdx, matches[i].queryIdx, matches[i].distance));
                    }
                }
            }else {
                return false;
            }
        }else if((_kf->featureCloud == nullptr || _kf->featureCloud->size() ==0)) { // No feature cloud
            auto t1 = std::chrono::high_resolution_clock::now();
            // Fine rotation.
            if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                return false;   // reject keyframe.
            }
            auto t2 = std::chrono::high_resolution_clock::now();

            std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
        }else { // Feature cloud and dense cloud
            // Compute initial rotation.
            if(!transformationBetweenFeatures( mLastKeyframe, _kf, transformation)){
                return false;   // reject keyframe.
            }

            if(mIcpEnabled){
                // Fine rotation.
                if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }
            }
        }

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::Affine3f prevPose(mLastKeyframe->pose);
        Eigen::Affine3f lastTransformation(transformation);
        // Compute current position.
        Eigen::Affine3f currentPose = prevPose*lastTransformation;

        // Check transformation
        Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
        float angleThreshold = 20.0;///180.0*M_PI;
        float distanceThreshold = 3.0;
        if((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3,1>(0,3).norm() > distanceThreshold){
            std::cout << "Large transformation! not accepted KF" << std::endl;
            return false;
        }

        _kf->position = currentPose.translation();
        _kf->orientation = currentPose.rotation();
        _kf->pose = currentPose.matrix();
    }

    // Add keyframe to list.
    mDatabase.addDataframe(_kf);

    // Set kf as last kf
    mLastKeyframe = _kf;
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline std::vector<std::shared_ptr<ClusterFrames<PointType_>>>  SceneRegistrator<PointType_>::clusters(){
    return mDatabase.clusters();
}

//-----------------------------------------------------------------------------------------------------------------
template<typename PointType_>
std::shared_ptr<DataFrame<PointType_>> SceneRegistrator<PointType_>::lastFrame() const{
    return mLastKeyframe;
}


//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::baMinError       () const{
    return mBA.minError();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline unsigned SceneRegistrator<PointType_>::baIterations     () const{
    return mBA.iterations();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline unsigned SceneRegistrator<PointType_>::baMinAparitions  () const{
    return mBA.minAparitions();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::descriptorDistanceFactor     () const{
    return mFactorDescriptorDistance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline int SceneRegistrator<PointType_>::ransacIterations             () const{
    return mRansacIterations;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::ransacMaxDistance   () const{
    return mRansacMaxDistance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline int SceneRegistrator<PointType_>::ransacMinInliers   () const{
    return mRansacMinInliers;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::icpMaxTransformationEpsilon() const{
    return mIcpMaxTransformationEpsilon;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance() const{
    return mIcpMaxCorrespondenceDistance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::icpVoxelDistance() const{
    return mIcpVoxelDistance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::icpMaxFitnessScore() const{
    return  mIcpMaxFitnessScore;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline int SceneRegistrator<PointType_>::icpMaxIterations() const{
    return mIcpMaxIterations;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::baMinError         (double _error){
    mBA.minError(_error);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::baIterations       (unsigned _iterations){
    mBA.iterations(_iterations);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::baMinAparitions    (unsigned _aparitions){
    mBA.minAparitions(_aparitions);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::descriptorDistanceFactor       (double _factor){
    mFactorDescriptorDistance = _factor;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::ransacIterations               (int _iterations){
    mRansacIterations = _iterations;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::ransacMaxDistance     (double _maxDistance){
    mRansacMaxDistance = _maxDistance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::ransacMinInliers     (int _minInliers){
    mRansacMinInliers = _minInliers;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpMaxTransformationEpsilon        (double _maxEpsilon){
    mIcpMaxTransformationEpsilon = _maxEpsilon;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance       (double _distance){
    mIcpMaxCorrespondenceDistance = _distance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpVoxelDistance     (double _distance){
    mIcpVoxelDistance = _distance;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpMaxFitnessScore (double _maxScore){
    mIcpMaxFitnessScore = _maxScore;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpMaxIterations (int _maxIters){
    mIcpMaxIterations = _maxIters;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
bool SceneRegistrator<PointType_>::initVocabulary(std::string _path){
    return mDatabase.initVocabulary(_path);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers) {
    std::vector<cv::DMatch> matches12, matches21;
    cv::BFMatcher featureMatcher;
    featureMatcher.match(_des1, _des2, matches12);
    featureMatcher.match(_des2, _des1, matches21);

    double max_dist = 0; double min_dist = 999999;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < _des1.rows; i++ ) {
        double dist = matches12[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    // symmetry test.
    for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
        for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
            if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                if(it12->distance <= min_dist*mFactorDescriptorDistance){
                    _inliers.push_back(*it12);
                }
                break;
            }
        }
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(std::shared_ptr<DataFrame<PointType_>> &_previousKf, std::shared_ptr<DataFrame<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
    if(_currentKf->multimatchesInliersKfs.find(_previousKf->id) !=  _currentKf->multimatchesInliersKfs.end()){
        // Match already computed
        std::cout << "Match alread computed between frames: " <<_currentKf->id << " and " << _previousKf->id << std::endl;
        return true;
    }
    std::vector<cv::DMatch> matches;
    matchDescriptors(_currentKf->featureDescriptors, _previousKf->featureDescriptors, matches);

    std::vector<int> inliers;
    rgbd::ransacAlignment<PointType_>(_currentKf->featureCloud, _previousKf->featureCloud, matches,_transformation, inliers, mRansacMaxDistance, mRansacIterations);

    if (inliers.size() >= 12) {
        _currentKf->multimatchesInliersKfs[_previousKf->id];
        _previousKf->multimatchesInliersKfs[_currentKf->id];
        int j = 0;
        for(int i = 0; i < inliers.size(); i++){
            while(matches[j].queryIdx != inliers[i]){
                j++;
            }
            _currentKf->multimatchesInliersKfs[_previousKf->id].push_back(matches[j]);
            _previousKf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

        }
        return true;
    }else{

        return false;
    }
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::refineTransformation(std::shared_ptr<DataFrame<PointType_>> &_previousKf, std::shared_ptr<DataFrame<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
    return icpAlignment<PointType_>(_currentKf->cloud,
                                    _previousKf->cloud,
                                    _transformation,
                                    mIcpMaxIterations,
                                    mIcpMaxCorrespondenceDistance,
                                    0.707,
                                    0.3,
                                    0.001,
                                    0.005,
                                    mIcpMaxFitnessScore);
}


//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::icpEnabled(bool _enable) {
    mIcpEnabled = _enable;
}


//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::icpEnabled() const {
    return mIcpEnabled;
}
}

