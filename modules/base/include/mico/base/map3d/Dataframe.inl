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

namespace mico {
    template<typename PointType_>
    inline Dataframe<PointType_>::Dataframe(size_t _id): id_(_id){

    }

    template<typename PointType_>
    inline Dataframe<PointType_>::~Dataframe(){
    }

    template<typename PointType_>
    inline int Dataframe<PointType_>::id() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return id_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::appendCovisibility(Dataframe<PointType_>::Ptr &_other){
        std::lock_guard<std::mutex> lock(dataLock_);
        if(std::find(covisibility_.begin(), covisibility_.end(), _other) == covisibility_.end()){
            // std::cout << "Adding covisivility : " <<_id  << " to " << id_ << std::endl;
            covisibility_.push_back(_other);
        }
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::updateMMI(int _dfId, int _cfId){
        std::lock_guard<std::mutex> lock(dataLock_);
        // crossReferencedInliers()[_cfId] = crossReferencedInliers()[_dfId];
        assert(false); //  666 DONT KNOW WHAT THE HELL IS THIS 999
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::addWord(const std::shared_ptr<Word<PointType_>> &_word){
        std::lock_guard<std::mutex> lock(dataLock_);
        wordsReference_[_word->id] = _word;
    }
 
    template<typename PointType_>
    inline void Dataframe<PointType_>::eraseWord(std::shared_ptr<Word<PointType_>> &_word){
        std::lock_guard<std::mutex> lock(dataLock_);
        wordsReference_.erase(_word->id);
    }


    template<typename PointType_>
    inline std::map<int, std::shared_ptr<Word<PointType_>>> Dataframe<PointType_>::words(){
        std::lock_guard<std::mutex> lock(dataLock_);
        return wordsReference_;
    }

    template<typename PointType_>
    inline std::shared_ptr<Word<PointType_>> Dataframe<PointType_>::word(int _id){
        std::lock_guard<std::mutex> lock(dataLock_);
        return wordsReference_[_id];
    }


    template<typename PointType_>
    inline void Dataframe<PointType_>::pose(const Eigen::Matrix4f &_pose){
        std::lock_guard<std::mutex> lock(dataLock_);
        pose_          = _pose;
        position_      = _pose.block<3,1>(0,3);
        orientation_   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Dataframe<PointType_>::pose() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return pose_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::cloud(const typename pcl::PointCloud<PointType_>::Ptr &_cloud){
        std::lock_guard<std::mutex> lock(dataLock_);
        cloud_ = _cloud;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr Dataframe<PointType_>::cloud() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return cloud_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureCloud(const typename pcl::PointCloud<PointType_>::Ptr &_cloud){
        std::lock_guard<std::mutex> lock(dataLock_);
        featureCloud_ = _cloud;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr Dataframe<PointType_>::featureCloud() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return featureCloud_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureDescriptors(const cv::Mat &_descriptors){
        std::lock_guard<std::mutex> lock(dataLock_);
        featureDescriptors_ = _descriptors;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::featureDescriptors() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return featureDescriptors_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureProjections(const std::vector<cv::Point2f> &_projs){
        std::lock_guard<std::mutex> lock(dataLock_);
        featureProjections_ = _projs;
    }

    template<typename PointType_>
    inline std::vector<cv::Point2f> Dataframe<PointType_>::featureProjections() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return featureProjections_;
    }

    template<typename PointType_>
    inline bool Dataframe<PointType_>::isOptimized() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return optimized_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::isOptimized(bool _opt){
        std::lock_guard<std::mutex> lock(dataLock_);
        optimized_ = _opt;
    }


    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::leftImage() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return left_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::leftImage(const cv::Mat &_image){
        std::lock_guard<std::mutex> lock(dataLock_);
        left_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::rightImage() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return right_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::rightImage(const cv::Mat &_image){
        std::lock_guard<std::mutex> lock(dataLock_);
        right_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::depthImage() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return depth_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::depthImage(const cv::Mat &_image){
        std::lock_guard<std::mutex> lock(dataLock_);
        depth_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::intrinsics() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return intrinsics_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::intrinsics(const cv::Mat &_intrinsics){
        std::lock_guard<std::mutex> lock(dataLock_);
        intrinsics_ = _intrinsics;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::distCoeff() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return coefficients_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::distCoeff(const cv::Mat &_coeff){
        std::lock_guard<std::mutex> lock(dataLock_);
        coefficients_ = _coeff;
    }


    template<typename PointType_>
    inline std::map<int, std::vector<cv::DMatch>> &Dataframe<PointType_>::crossReferencedInliers(){
        std::lock_guard<std::mutex> lock(dataLock_);
        return multimatchesInliersDfs_;
    }


    template<typename PointType_>
    inline std::vector<typename  Dataframe<PointType_>::Ptr> Dataframe<PointType_>::covisibility(){
        std::lock_guard<std::mutex> lock(dataLock_);
        return covisibility_;
    }

    #ifdef USE_DBOW2
    template<typename PointType_>
    inline void Dataframe<PointType_>::signature(DBoW2::BowVector &_signature){
        std::lock_guard<std::mutex> lock(dataLock_);
        signature_ = _signature;
    }

    template<typename PointType_>
    inline DBoW2::BowVector Dataframe<PointType_>::signature() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return signature_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureVector(DBoW2::FeatureVector &_vector){
        std::lock_guard<std::mutex> lock(dataLock_);
        featVec_ = _vector;
    }

    template<typename PointType_>
    inline DBoW2::FeatureVector Dataframe<PointType_>::featureVector() const{
        std::lock_guard<std::mutex> lock(dataLock_);
        return featVec_;
    }
    #endif

    template<typename PointType_>
    inline void Dataframe<PointType_>::wordCreation(){
        assert(covisibility_.size() == 1);  // Just one KF In covisibility.

        auto prevDf = covisibility_[0];
        auto selfRef = Dataframe<PointType_>::Ptr(this);

        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*prevDf->featureCloud(), *transformedFeatureCloud, prevDf->pose());
        std::vector<cv::DMatch> cvInliers = selfRef->crossReferencedInliers()[prevDf->id()];

        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++){
            // 666 Assumes that is only matched with previous cloud, loops are not handled in this method
            std::shared_ptr<Word<PointType_>> prevWord = nullptr;
            int inlierIdxInDataframe = cvInliers[inlierIdx].queryIdx;
            int inlierIdxInCluster = cvInliers[inlierIdx].trainIdx;

            // Check if exists a word with the id of the descriptor inlier
            for (auto &w : /*mWordDictionary*/ prevDf->words()){ // TODO: Can we check if the word have current dataframe id as key?
                if (w.second->idxInDf[prevDf->id()] == inlierIdxInCluster) {
                    prevWord = w.second;
                    break;
                }
            }
            if (prevWord) {
                //Cluster
                if (prevWord->dfMap.find(selfRef->id()) == prevWord->dfMap.end()) {
                    std::vector<float> projection = {   selfRef->featureProjections()[inlierIdxInDataframe].x,
                                                        selfRef->featureProjections()[inlierIdxInDataframe].y};
                    prevWord->addObservation(selfRef, inlierIdxInDataframe, projection);
                    selfRef->addWord(prevWord);
                    
                    // 666 CHECK IF IT IS NECESARY
                    for (auto &df : prevWord->dfMap) {
                        selfRef->appendCovisibility(df.second);
                        // Add current dataframe id to others dataframe covisibility
                        df.second->appendCovisibility(selfRef);
                    }
                }
            }
            else {
                // Create word
                int wordId = 0;
                if(prevDf->words().size()>0)
                    wordId = prevDf->words().rbegin()->first+1;    // THE HELLL

                auto pclPoint = (*transformedFeatureCloud)[inlierIdxInCluster];
                std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
                auto descriptor = prevDf->featureDescriptors().row(inlierIdxInCluster);

                auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));

                //Add word to current dataframe
                selfRef->addWord(newWord);

                // Add word to new dataframe (new dataframe is representative of the new dataframe)
                std::vector<float> dataframeProjections = { selfRef->featureProjections()[inlierIdxInDataframe].x, 
                                                            selfRef->featureProjections()[inlierIdxInDataframe].y};
                newWord->addObservation(selfRef, inlierIdxInDataframe, dataframeProjections);

                selfRef->appendCovisibility(prevDf);
                selfRef->addWord(newWord);

                // Add word to last dataframe
                std::vector<float> projection = {   prevDf->featureProjections()[inlierIdxInCluster].x, 
                                                    prevDf->featureProjections()[inlierIdxInCluster].y};
                newWord->addObservation(prevDf, inlierIdxInCluster, projection);
                prevDf->appendCovisibility(prevDf);
                prevDf->addWord(newWord);
            }
        }
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::reinforce(std::shared_ptr<Dataframe<PointType_>> &_df){
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        if (!transformationBetweenFeatures( _df, 
                                            Dataframe<PointType_>::Ptr(this), 
                                            transformation,
                                            1, 
                                            0.03,
                                            1000, 
                                            10,
                                            25.0 /* Descriptor distance factor*/ , 
                                            10)) //TODO: json parameters
        { 
            std::cout << "DatabaseMarkI, <10 inliers between df: " + std::to_string(_df->id()) + " and df " + std::to_string(id_) + " " << std::endl;
        }else{
            std::cout << "DatabaseMarkI, comparison between df: " + std::to_string(_df->id()) + " and df " + std::to_string(id_) << std::endl;
            reinforceWords(_df, Dataframe<PointType_>::Ptr(this));
        }
    }


    template<typename PointType_>
    inline void Dataframe<PointType_>::reinforceWords(  std::shared_ptr<mico::Dataframe<PointType_>> _df){

        // typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        // pcl::transformPointCloud(*this->featureCloud(), *transformedFeatureCloud, this->pose());
        
        // std::vector<cv::DMatch> cvInliers = _df->crossReferencedInliers()[this->id()];

        // // Word count
        // int newWords = 0;
        // int duplicatedWords = 0; 
        // int wordsOnlyInOneCluster =0; 
        // int goodWords = 0;
        // for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++) { 
        //     int inlierIdxInQuery = cvInliers[inlierIdx].queryIdx;
        //     int inlierIdxInTrain = cvInliers[inlierIdx].trainIdx;

        //     std::shared_ptr<Word<PointType_>> trainWord = nullptr;
        //     // Check if exists a word with the id of the descriptor inlier in the train dataframe
        //     for (auto &w : this->words())       {
        //         if (w.second->idxInDf[this->id()] == inlierIdxInTrain) {
        //             trainWord = w.second;
        //             break;
        //         }
        //     }
        //     // Check if exists a word with the id of the descriptor inlier in the query dataframe
        //     std::shared_ptr<Word<PointType_>> queryWord = nullptr;
        //     for (auto &w : _df->words())    {
        //         if (w.second->idxInDf[_df->id()] == inlierIdxInQuery) {
        //             queryWord = w.second;
        //             break;
        //         }
        //     }

        //     if(queryWord){
        //         if(trainWord){    
        //             if(trainWord->id != queryWord->id){
        //                 // Merge words. Erase newest word
        //                 trainWord->mergeWord(queryWord);
        //                 //Delete word from mWordDictionary
        //                 mWordDictionary.erase(queryWord->id);
        //                 // Add word
        //                 _df->addWord(trainWord);
        //                 duplicatedWords++;
        //             }
        //             else{
        //                 goodWords++;
        //             }
        //         }else{
        //             // Add info of queryWord in train dataframe and update queryWord
        //             std::vector<float> trainProjections = { this->featureProjections()[inlierIdxInTrain].x, 
        //                                                     this->featureProjections()[inlierIdxInTrain].y};
        //             queryWord->addObservation(this, inlierIdxInTrain, trainProjections);
        //             this->addWord(queryWord);

        //             // Update covisibility
        //             this->appendCovisibility(_df);
        //             _df->appendCovisibility(this);

        //             wordsOnlyInOneCluster++;
        //         }     
        //     }else{
        //         if(trainWord){
        //             // Add info of trainWord in query dataframe and update trainWord
        //             std::vector<float> queryProjections = { _df->featureProjections()[inlierIdxInQuery].x, 
        //                                                     _df->featureProjections()[inlierIdxInQuery].y};
        //             trainWord->addObservation(_df, inlierIdxInQuery, queryProjections);
        //             _df->addWord(trainWord);

        //             // Update covisibility
        //             this->appendCovisibility(_df);
        //             _df->appendCovisibility(this);
        //             wordsOnlyInOneCluster++;
        //         }else{
        //             // New word
        //             int wordId = mWordDictionary.rbegin()->first+1;  aqwasdaweasdaweasdqweasd
        //             auto pclPoint = (*transformedFeatureCloud)[inlierIdxInTrain];   // 3D point of the trainDf
        //             std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
        //             auto descriptor = this->featureDescriptors().row(inlierIdxInTrain);
        //             auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));
                    
        //             // Add word to train dataframe
        //             std::vector<float> trainProjections = { this->featureProjections()[inlierIdxInTrain].x, 
        //                                                     this->featureProjections()[inlierIdxInTrain].y};
        //             newWord->addObservation(this, inlierIdxInTrain, trainProjections);
        //             this->appendCovisibility(_df);
        //             this->addWord(newWord);

        //             // Add word to query dataframe
        //             std::vector<float> queryProjections = { _df->featureProjections()[inlierIdxInQuery].x, 
        //                                                     _df->featureProjections()[inlierIdxInQuery].y};
        //             newWord->addObservation(_df, inlierIdxInQuery, queryProjections);
        //             _df->appendCovisibility(this);
        //             _df->addWord(newWord);

        //             // Add word to dictionary
        //             mWordDictionary[wordId] = newWord;

        //             newWords++;
        //         }
        //     }
        // }
    }

}