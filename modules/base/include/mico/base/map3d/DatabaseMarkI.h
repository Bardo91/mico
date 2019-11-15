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

#ifndef MICO_BASE_MAP3D_DATABASE_MARKI_H_
#define MICO_BASE_MAP3D_DATABASE_MARKI_H_

#include <mico/base/map3d/Dataframe.h>
#include <mico/base/utils/LogManager.h>
#include <mico/base/cjson/json.h>


namespace mico {
    template <typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    class DatabaseMarkI : public LoggableInterface<DebugLevel_, OutInterface_>{
    public:
        /// \brief init vocabulary
        /// \param _path: path to file containing the vocabulary
        /// \return true if initialized, false if not.
        bool init(const cjson::Json &_configFile);

        /// Compute two keyframes to get his transform and matches
        bool addDataframe(std::shared_ptr<mico::Dataframe<PointType_>> _df);

        std::shared_ptr<mico::Dataframe<PointType_>> lastDataframe(){
            return mLastDataframe;
        }

    private:
        /// Create new ClusterFrame
        bool updateCurrentKeyframe(std::shared_ptr<mico::Dataframe<PointType_>> _df);

        /// Score between current DF and new DF
        double computeScore(std::shared_ptr<mico::Dataframe<PointType_>> _df1,
                            std::shared_ptr<mico::Dataframe<PointType_>> _df2);

        /// Write signature of a df from its words
        void writeSignature(std::shared_ptr<mico::Dataframe<PointType_>> &_df);

        /// Find and create words comparing current dataframe with last dataframe added
        void wordCreation(  std::shared_ptr<mico::Dataframe<PointType_>> _prevDf,
                            std::shared_ptr<mico::Dataframe<PointType_>> _currentDf);

        /// Find and create words comparing two df
        void wordComparison(std::shared_ptr<mico::Dataframe<PointType_>> _queryCluster,
                            std::shared_ptr<mico::Dataframe<PointType_>> _trainCluster);

        /// Check duplicated words beetween near df
        void dfComparison(std::map<int,std::shared_ptr<Dataframe<PointType_>>> _dfSet, bool _localComparison);
        
        std::shared_ptr<mico::Dataframe<PointType_>> mLastDataframe=nullptr;
        std::shared_ptr<Word<PointType_>> mLastWord=nullptr;
        
        double mScore=0.4;
        int mNumCluster = 0; 
        #ifdef USE_DBOW2
            OrbVocabulary mVocabulary;
        #endif
    };

} // namespace mico 

#include "DatabaseMarkI.inl"

#endif // 