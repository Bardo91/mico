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

#include <mico/base/map3d/utils3d.h>
#include <mico/base/utils/LogManager.h>

#include <mico/base/map3d/Word.h>

#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <iostream>
#include <memory>


namespace mico {

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::init(const cjson::Json &_configFile) {
        mScore = ((double)_configFile["similarity_score"]);
        
        if(_configFile.contains("clusterComparison")) 
            mNumCluster = (int)_configFile["clusterComparison"];
            
        #ifdef USE_DBOW2
            if(_configFile.contains("vocabulary")) 
                mVocabulary.load(_configFile["vocabulary"]);
            //mVocabulary.setScoringType(DBoW2::L2_NORM);  //TODO: Change this scoring type
            return !mVocabulary.empty();
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::addDataframe(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        
        writeSignature(_df);
        double score = computeScore(mLastDataframe, _df);
        
        if(score > mScore){
            return false;
        }
        
        if(mLastDataframe){
            _df->appendCovisibility(mLastDataframe);
            _df->wordCreation(_df, mLastDataframe);
        }

        updateCurrentKeyframe(_df);
        
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::writeSignature(std::shared_ptr<mico::Dataframe<PointType_>> &_df) {
        std::vector<cv::Mat> descriptors;
        for (auto &w : _df->words()) {
            descriptors.push_back(w.second->descriptor);
        }

        DBoW2::BowVector signature;
        DBoW2::FeatureVector featVec; 
        mVocabulary.transform(descriptors,signature, featVec, 4);
        _df->signature(signature);
        _df->featureVector(featVec);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::computeScore(  std::shared_ptr<mico::Dataframe<PointType_>> _df1,
                                                                                        std::shared_ptr<mico::Dataframe<PointType_>> _df2) {
        if(_df1 == nullptr || _df2 == nullptr)
            return 0;

        #ifdef USE_DBOW2
            // Adding df in current dataframe or create a new one
            return mVocabulary.score(_df1->signature(), _df2->signature());
        #else
            return 0;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::updateCurrentKeyframe(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        

        // Update MMI of previous daraframe
        // if (mDataframes.size() > 1){
        //     if(mNumCluster>1){
        //         int n = 0;
        //         // local dataframe comparison
        //         std::map<int, std::shared_ptr<Dataframe<PointType_>>> localSubset;
        //         localSubset[mLastDataframe->id()] = mLastDataframe;
        //         for (auto trainDf = mDataframes.rbegin(); trainDf != mDataframes.rend() && n <= mNumCluster +2; trainDf++, n++)
        //         {   
        //             localSubset[trainDf->first] = trainDf->second;
        //         }
        //         dfComparison(localSubset,true);
        //     }
        // }

        mLastDataframe = _df;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::dfComparison(std::map<int,std::shared_ptr<Dataframe<PointType_>>> _dfSet, bool _localComparison)
    {
        for (auto queryDf = _dfSet.rbegin(); queryDf != _dfSet.rend(); queryDf++){
            for (auto trainDf = _dfSet.begin(); (trainDf != _dfSet.end() && (queryDf->first) > (trainDf->first)); trainDf++){
                trainDf->reinforce(queryDf);
            }
            if(_localComparison)
                break;
        }
    }

} // namespace mico 