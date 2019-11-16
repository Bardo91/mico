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

#include <unordered_map>

#include <mico/base/utils/Gui.h>

#include <pcl/common/transforms.h>

#include <mico/base/map3d/Word.h>

namespace mico {
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minError       () const{
        return mBaMinError;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline unsigned BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::iterations     () const{
        return mBaIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline unsigned BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minWords     () const{
        return mMinWords;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minError         (double _error){
        mBaMinError = _error;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minWords         (unsigned _minWords){
        mMinWords = _minWords;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::iterations       (unsigned _iterations){
        mBaIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minAparitions       (unsigned  _aparitions){
        mBaMinAparitions = _aparitions;
    }

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::sequence(std::map<int,std::shared_ptr<Dataframe<PointType_>>> &_dataframes){
        this->status("BA","Cleaning old data");
        cleanDataParent();

        mDataframes = _dataframes;
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::prepareDataClusterframes(){
        this->status("BA","Preparing data");
        unsigned nWords = 0;
        for(auto &df: mDataframes){
            for(auto  &word: df.second->words()){
                if(!mUsedWordsMap[word.second->id] &&  word.second->dfMap.size() > this->mBaMinAparitions){
                    nWords++;
                    mUsedWordsMap[word.second->id] = true;  // check true to use it later
                    mGlobalUsedWordsRef[word.second->id] = word.second;
                }
            }
        }
        
        this->status("BA","Found " + std::to_string(nWords) + " that exists in at least "+std::to_string(this->mBaMinAparitions)+" dataframes");
        
        if(nWords < mMinWords){
            this->warning("BA", "Not enough words to perform optimization");
            return false;
        }

        this->status("BA","Copying poses and camera data");
        int nFrames = mDataframes.size();
        reserveData(nFrames, nWords);

        int cameraId = 0;
        for(auto &df:mDataframes){
            cv::Mat intrinsics, coeffs;
            df.second->intrinsics().convertTo(intrinsics, CV_64F);
            df.second->distCoeff().convertTo(coeffs, CV_64F);
            
            appendCamera(cameraId, df.second->pose(), intrinsics, coeffs);

            mClustersIdToCameraId[df.second->id()] = cameraId;
            mCameraIdToClustersId[cameraId] = df.second->id();
            
            cameraId++;
        }

        this->status("BA","Copying words' position and projections");
    
        int pointId = 0;
        for(auto &df:mDataframes){
            cv::Mat intrinsics, coeffs;
            df.second->intrinsics().convertTo(intrinsics, CV_64F);
            df.second->distCoeff().convertTo(coeffs, CV_64F);
            for(auto &word: df.second->words()){
                if(!mUsedWordsMap[word.second->id])
                    continue;

                mUsedWordsMap[word.second->id] = false; // check false to prevent its use

                appendPoint(pointId, {word.second->point[0], word.second->point[1], word.second->point[2]});
                
                mWordIdToPointId[word.second->id] = pointId;
                mPointIdToWordId[pointId] = word.second->id;

                for(auto &dfPair: word.second->dfMap){
                    if(mClustersIdToCameraId.find(dfPair.first) != mClustersIdToCameraId.end()){
                        int cameraId = mClustersIdToCameraId[dfPair.first];
                        if(word.second->projectionsEnabled[dfPair.first]){  
                            appendProjection(cameraId, pointId, word.second->cvProjectiond(dfPair.first), intrinsics, coeffs);
                        }else{
                            this->warning("BA", "Projection of word " +std::to_string(word.second->id) + " in df " + std::to_string(dfPair.first) + " is not enabled");
                        }            
                    }
                }
                pointId++;
            }
        }

        fitSize(mDataframes.size(), pointId);

        return true;
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::cleanDataParent(){
        mDataframes.clear();
        mUsedWordsMap.clear();   // 666 placed  here to prevent weird memory crash.

        mClustersIdToCameraId.clear();
        mCameraIdToClustersId.clear();
        mWordIdToPointId.clear();
        mPointIdToWordId.clear();
        mGlobalUsedWordsRef.clear();
        
        this->cleanData();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::optimize(){
        this->status("BA","Optimizing " + std::to_string(mDataframes.size()) + " dataframes");
        
        if(!prepareDataClusterframes()){
            this->warning("BA", "Failed data preparation");    
            return false;
        }

        checkData();

        this->status("BA", "Init optimization");
        
        if(!doOptimize()){
            this->error("BA", "Failed Optimization");
            return false;
        }

        this->status("BA", "Optimized, recovering data");

        // Recovering df data
        for(auto &pairCamera : mCameraIdToClustersId){
            Eigen::Matrix4f pose;
            cv::Mat intrinsics, coeffs;
            this->status("BA","Recovering camera "+std::to_string(pairCamera.first)+", which is df  "+std::to_string(pairCamera.second)+". Of a total of "+std::to_string(mCameraIdToClustersId.size())+ " cameras.");
            recoverCamera(pairCamera.first, pose, intrinsics, coeffs);

            mDataframes[pairCamera.second]->pose(pose);

            mDataframes[pairCamera.second]->isOptimized(true);
        }

        // recovering points
        for(auto &pairPoint : mPointIdToWordId){
            cv::Mat intrinsics, coeffs;
            Eigen::Vector3f position;

            recoverPoint(pairPoint.first, position);

            auto word = mGlobalUsedWordsRef[pairPoint.second];
            word->point = {
                position[0],
                position[1],
                position[2]
            };
            word->optimized = true;

            for(auto &dfPair: word->dfMap){
                if(mClustersIdToCameraId.find(dfPair.first) != mClustersIdToCameraId.end()){
                      int cameraId = mClustersIdToCameraId[dfPair.first];
                    if(!isProjectionEnabled(dfPair.first, pairPoint.first)){
                        mGlobalUsedWordsRef[pairPoint.second]->projectionsEnabled[dfPair.first] = false;
                        this->warning("BA", "Dropping edge (camera, point): ("+std::to_string(dfPair.first)+", "+std::to_string(pairPoint.second)+")");
                    }
                }
            }

        }

        this->status("BA", "Data restored");
        return true;
    }
}
