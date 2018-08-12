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


#include <rgbd_tools/map3d/cvsba/cvsba.h>
#include <unordered_map>
#include <rgbd_tools/utils/Gui.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <iterator>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::optimize() {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::optimizeClusterframe() {
        this->status("BA_CVSBA", "Preparing data");
        prepareDataCluster();
        
        this->status("BA_CVSBA", "Init optimization");
        // Initialize cvSBA and perform bundle adjustment.
        cvsba::Sba bundleAdjuster;
        cvsba::Sba::Params params;
        params.verbose = true;
        params.iterations = this->mBaIterations;
        params.minError = this->mBaMinError;
        params.type = cvsba::Sba::MOTIONSTRUCTURE;
        bundleAdjuster.setParams(params);

        assert(mScenePoints.size() == mScenePointsProjection[0].size());
        assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
        assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mCoeffs.size());
        assert(mIntrinsics.size() == mRotations.size());
        assert(mTranslations.size() == mRotations.size());


        Eigen::Matrix4f pose01 = mClusterframe->dataframes[mClusterframe->frames[0]]->pose;
        bundleAdjuster.run(mScenePoints, mScenePointsProjection, mCovisibilityMatrix, mIntrinsics, mRotations, mTranslations, mCoeffs);

        this->status("BA_CVSBA", "Optimized");
        Eigen::Matrix4f incPose;
        for(unsigned i = 0; i < mTranslations.size(); i++){
            cv::Mat R = mRotations[i];
            Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();

            newPose(0,0) = R.at<double>(0,0);
            newPose(0,1) = R.at<double>(0,1);
            newPose(0,2) = R.at<double>(0,2);
            newPose(1,0) = R.at<double>(1,0);
            newPose(1,1) = R.at<double>(1,1);
            newPose(1,2) = R.at<double>(1,2);
            newPose(2,0) = R.at<double>(2,0);
            newPose(2,1) = R.at<double>(2,1);
            newPose(2,2) = R.at<double>(2,2);
        
            newPose(0,3) = mTranslations[i].at<double>(0);
            newPose(1,3) = mTranslations[i].at<double>(1);
            newPose(2,3) = mTranslations[i].at<double>(2);

            if(i == 0){
                Eigen::Matrix4f pose02 = newPose;
                incPose = pose02.inverse()*pose01;
            }

            newPose = incPose*newPose;

            mClusterframe->dataframes[mClusterframe->frames[i]]->updatePose(newPose);
        }


        for(unsigned i = 0; i < mIdxToId.size(); i++){
            int id = mIdxToId[i];
            mClusterframe->wordsReference[id]->point = {
                mScenePoints[i].x,
                mScenePoints[i].y,
                mScenePoints[i].z
            };
            mClusterframe->wordsReference[id]->optimized = true;
        }
        this->status("BA_CVSBA", "Data restored");
        
        return true;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes) {
        mKeyframes = _keyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::keyframes(typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end) {
       
    }

    //---------------------------------------------------------------------------------------------------------------------
    // Not implemented yet.
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::clusterframe(std::shared_ptr<ClusterFrames<PointType_>> &_clusterframe) {
        mClusterframe = _clusterframe;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    std::vector<DataFrame<PointType_> , Eigen::aligned_allocator <DataFrame<PointType_>>> BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::keyframes() {
        return mKeyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::cleanData() {
        mCovisibilityMatrix.clear();
        mScenePoints.clear();
        mScenePointsProjection.clear();
        mIntrinsics.clear();
        mCoeffs.clear();
        mTranslations.clear();
        mRotations.clear();
        mIdxToId.clear();
        mUsedWordsMap.clear();
        mClustersIdxToId.clear();
        mGlobalUsedWordsRef.clear();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::prepareData() {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::prepareDataCluster() {
        cleanData();

        int nWords = mClusterframe->wordsReference.size();
        int nFrames = mClusterframe->dataframes.size();
        mCovisibilityMatrix.resize(nFrames);
        mScenePointsProjection.resize(nFrames);

        for(unsigned i = 0; i < nFrames; i++){
            mCovisibilityMatrix[i].resize(nWords, 0);
            mScenePointsProjection[i].resize(   nWords, 
                                                cv::Point2d(    std::numeric_limits<double>::quiet_NaN(),
                                                                std::numeric_limits<double>::quiet_NaN()));

            cv::Mat intrinsics, coeffs;
            mClusterframe->intrinsic.convertTo(intrinsics, CV_64F);
            mClusterframe->distCoeff.convertTo(coeffs, CV_64F);

            mIntrinsics.push_back(intrinsics.clone());
            mCoeffs.push_back(coeffs.clone());

            int frameId  = mClusterframe->frames[i];
            Eigen::Matrix4f poseInv = mClusterframe->dataframes[frameId]->pose;

            cv::Mat cvRotation(3,3,CV_64F);
            cvRotation.at<double>(0,0) = poseInv(0,0);
            cvRotation.at<double>(0,1) = poseInv(0,1);
            cvRotation.at<double>(0,2) = poseInv(0,2);
            cvRotation.at<double>(1,0) = poseInv(1,0);
            cvRotation.at<double>(1,1) = poseInv(1,1);
            cvRotation.at<double>(1,2) = poseInv(1,2);
            cvRotation.at<double>(2,0) = poseInv(2,0);
            cvRotation.at<double>(2,1) = poseInv(2,1);
            cvRotation.at<double>(2,2) = poseInv(2,2);
            mRotations.push_back(cvRotation.clone());

            cv::Mat cvTrans(3,1,CV_64F);
            cvTrans.at<double>(0) = poseInv(0,3);
            cvTrans.at<double>(1) = poseInv(1,3);
            cvTrans.at<double>(2) = poseInv(2,3);
            mTranslations.push_back(cvTrans.clone());

        }

        mScenePoints.resize(nWords);
        for(auto &word: mClusterframe->wordsReference){
            if(word.second->frames.size() > this->mBaMinAparitions){
                int idx = mIdxToId.size();
                mScenePoints[idx] = cv::Point3d(word.second->point[0], word.second->point[1], word.second->point[2]);

                for(auto &frame: word.second->frames){
                    auto frameIter = std::find(mClusterframe->frames.begin(), mClusterframe->frames.end(), frame);
                    if(frameIter != mClusterframe->frames.end()){
                        int index = std::distance(mClusterframe->frames.begin(), frameIter);
                        mScenePointsProjection[index][idx].x = word.second->projections[frame][0];
                        mScenePointsProjection[index][idx].y = word.second->projections[frame][1];
                        mCovisibilityMatrix[index][idx] = 1;
                    }
                }

                mIdxToId.push_back(word.second->id);
            }
        }

        mScenePoints.resize(mIdxToId.size());
        for(auto&visibility:mCovisibilityMatrix){
            visibility.resize(mIdxToId.size());
        }
        for(auto&projections:mScenePointsProjection){
            projections.resize(mIdxToId.size());
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::clusterframes(std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> &_clusterframes){
        this->status("BA_CVSBA","Cleaning old data");
        cleanData();

        mClusterFrames = _clusterframes;
    };
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::prepareDataClusters(){
        this->status("BA_CVSBA","Preparing data");
        int nWords = 0;
        for(auto &cluster: mClusterFrames){
            auto bestDataframe = cluster.second->dataframes[cluster.second->bestDataframe];
            for(auto  &word: bestDataframe->wordsReference){
                if(!mUsedWordsMap[word->id] &&  word->clusters.size() > this->mBaMinAparitions){
                    nWords++;
                    mUsedWordsMap[word->id] = true;  // check true to use it later
                    mGlobalUsedWordsRef[word->id] = word;
                }
            }
        }
        
        this->status("BA_CVSBA","Found " + std::to_string(nWords) + " that exists in at least 2 clusters");
        
        this->status("BA_CVSBA","Copying poses and camera data");
        int nFrames = mClusterFrames.size();
        mCovisibilityMatrix.resize(nFrames);
        mScenePointsProjection.resize(nFrames);
        mRotations.resize(nFrames);
        mTranslations.resize(nFrames);
        mIntrinsics.resize(nFrames);
        mCoeffs.resize(nFrames);

        int clusterIdx = 0;
        for(auto &cluster:mClusterFrames){
            mCovisibilityMatrix[clusterIdx].resize(nWords, 0);
            mScenePointsProjection[clusterIdx].resize(   nWords, 
                                                cv::Point2d(    std::numeric_limits<double>::quiet_NaN(),
                                                                std::numeric_limits<double>::quiet_NaN()));

            cv::Mat intrinsics, coeffs;
            cluster.second->intrinsic.convertTo(intrinsics, CV_64F);
            cluster.second->distCoeff.convertTo(coeffs, CV_64F);

            mIntrinsics[clusterIdx] = intrinsics.clone();
            mCoeffs[clusterIdx] = coeffs.clone();

            Eigen::Matrix4f poseInv = cluster.second->bestPose();

            cv::Mat cvRotation(3,3,CV_64F);
            cvRotation.at<double>(0,0) = poseInv(0,0);
            cvRotation.at<double>(0,1) = poseInv(0,1);
            cvRotation.at<double>(0,2) = poseInv(0,2);
            cvRotation.at<double>(1,0) = poseInv(1,0);
            cvRotation.at<double>(1,1) = poseInv(1,1);
            cvRotation.at<double>(1,2) = poseInv(1,2);
            cvRotation.at<double>(2,0) = poseInv(2,0);
            cvRotation.at<double>(2,1) = poseInv(2,1);
            cvRotation.at<double>(2,2) = poseInv(2,2);
            mRotations[clusterIdx] = cvRotation.clone();

            cv::Mat cvTrans(3,1,CV_64F);
            cvTrans.at<double>(0) = poseInv(0,3);
            cvTrans.at<double>(1) = poseInv(1,3);
            cvTrans.at<double>(2) = poseInv(2,3);
            mTranslations[clusterIdx] = cvTrans.clone();
            
            mClustersIdxToId.push_back(cluster.second->id);
            clusterIdx++;
        }

        this->status("BA_CVSBA","Copying words' position and projections");
        mScenePoints.resize(nWords);
        
        clusterIdx = 0;
        int wordsCounter = 0;
        mIdxToId.resize(nWords);
        for(auto &cluster:mClusterFrames){
            int bestDataframeId = cluster.second->bestDataframe;
            for(auto &word: cluster.second->bestDataframePtr()->wordsReference){
                if(!mUsedWordsMap[word->id])
                    continue;

                mUsedWordsMap[word->id] = false; // check false to prevent its use

                int idx = wordsCounter;
                int id = word->id;
                mScenePoints[idx] = cv::Point3d(word->point[0], word->point[1], word->point[2]);
                
                std::cout << "Word " << word->id; 
                for(auto &usedClusterId: word->clusters){
                    auto iterIdCluster = std::find(mClustersIdxToId.begin(), mClustersIdxToId.end(), usedClusterId);
                    auto bestDfIdInCluster = mClusterFrames[*iterIdCluster]->bestDataframe;
                    if(std::find(word->frames.begin(), word->frames.end(), bestDfIdInCluster) == word->frames.end())
                        continue;   // Word can be in cluster but not in best DF of cluster, so rejecting!.

                    if( iterIdCluster != mClustersIdxToId.end()){
                        int index = iterIdCluster - mClustersIdxToId.begin();
                        std::cout << " (" << *iterIdCluster << "," << bestDfIdInCluster<<"),";
                        mScenePointsProjection[index][idx].x 
                                            = 
                                            word->projections[bestDfIdInCluster][0];
                        mScenePointsProjection[index][idx].y 
                                            = 
                                            word->projections[bestDfIdInCluster][1];
                        mCovisibilityMatrix[index][idx] = 1;
                        mIdxToId[wordsCounter] = id;
                    }
                }
                std::cout << std::endl;
                wordsCounter++;
            }
            clusterIdx++;
        }

        mScenePoints.resize(wordsCounter);
        for(auto&visibility:mCovisibilityMatrix){
            visibility.resize(wordsCounter);
        }
        for(auto&projections:mScenePointsProjection){
            projections.resize(wordsCounter);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::optimizeClusterframes(){
        this->status("BA_CVSBA","Optimizing " + std::to_string(mClusterFrames.size()) + " cluster frames");
        prepareDataClusters();

        this->status("BA_CVSBA", "Init optimization");
        // Initialize cvSBA and perform bundle adjustment.
        cvsba::Sba bundleAdjuster;
        cvsba::Sba::Params params;
        params.verbose = true;
        params.iterations = this->mBaIterations;
        params.minError = this->mBaMinError;
        params.type = cvsba::Sba::MOTIONSTRUCTURE;
        bundleAdjuster.setParams(params);

        assert(mScenePoints.size() == mScenePointsProjection[0].size());
        assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
        assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mCoeffs.size());
        assert(mIntrinsics.size() == mRotations.size());
        assert(mTranslations.size() == mRotations.size());

        bundleAdjuster.run(mScenePoints, mScenePointsProjection, mCovisibilityMatrix, mIntrinsics, mRotations, mTranslations, mCoeffs);

        this->status("BA_CVSBA", "Optimized");
        Eigen::Matrix4f pose01 = mClusterFrames[mClustersIdxToId[0]]->bestDataframePtr()->pose;
        Eigen::Matrix4f incPose = Eigen::Matrix4f::Identity();
        for(unsigned i = 0; i < mTranslations.size(); i++){
            Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
            
            newPose(0,0) = mRotations[i].at<double>(0,0);
            newPose(0,1) = mRotations[i].at<double>(0,1);
            newPose(0,2) = mRotations[i].at<double>(0,2);
            newPose(1,0) = mRotations[i].at<double>(1,0);
            newPose(1,1) = mRotations[i].at<double>(1,1);
            newPose(1,2) = mRotations[i].at<double>(1,2);
            newPose(2,0) = mRotations[i].at<double>(2,0);
            newPose(2,1) = mRotations[i].at<double>(2,1);
            newPose(2,2) = mRotations[i].at<double>(2,2);
            
            newPose(0,3) = mTranslations[i].at<double>(0);
            newPose(1,3) = mTranslations[i].at<double>(1);
            newPose(2,3) = mTranslations[i].at<double>(2);
            
            if(i == 0){
                Eigen::Matrix4f pose02 = newPose;
                incPose = pose02.inverse()*pose01;
            }

            newPose = incPose*newPose;

            auto cluster = mClusterFrames[mClustersIdxToId[i]]; 

            Eigen::Matrix4f offsetCluster = cluster->bestDataframePtr()->pose.inverse()*newPose;
            
            cluster->bestDataframePtr()->updatePose(newPose);

            for(auto &df : cluster->dataframes){
                if(df.second->id != cluster->bestDataframe){
                    Eigen::Matrix4f updatedPose = offsetCluster*df.second->pose;
                    df.second->updatePose(updatedPose);
                }
            }
        }

        


        for(unsigned i = 0; i < mIdxToId.size(); i++){
            int id = mIdxToId[i];
            mGlobalUsedWordsRef[id]->point = {
                mScenePoints[i].x,
                mScenePoints[i].y,
                mScenePoints[i].z
            };
            mGlobalUsedWordsRef[id]->optimized = true;
        }
        this->status("BA_CVSBA", "Data restored");
        

        return true;
    }
}
