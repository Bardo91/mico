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


#ifndef MICO_BASE_MAP3D_DATAFRAME_H_
#define MICO_BASE_MAP3D_DATAFRAME_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

#include <Eigen/Eigen>

#include <map>

namespace mico {
    template<typename PointType_>
    class Word;

    template<typename PointType_>
    class Dataframe{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        /// Public type of shared_ptr of Dataframe.
        typedef std::shared_ptr<Dataframe<PointType_>> Ptr;

        /// Dataframes can only be created with ID which uniquely define it.
        Dataframe(size_t _id);
        ~Dataframe(){}

        int id() const;

        /// Add a new word to the internal map of words.
        void addWord(const std::shared_ptr<Word<PointType_>> &_word);

        /// Delete existing word in the internal map of words.
        void eraseWord(std::shared_ptr<Word<PointType_>> &_word);
        
        std::shared_ptr<Word<PointType_>> word(int _id);
        std::map<int, std::shared_ptr<Word<PointType_>>> words();

        /// Update multimatchesInliersCfs when a df becomes a cluster.
        void updateMMI(int _dfId, int _cfId);
 
        std::vector<typename Dataframe<PointType_>::Ptr> covisibility();

        /// Add the ID of another DF which is observed from this one
        void appendCovisibility(Dataframe<PointType_>::Ptr &_other);

        /// Set the pose of the dataframe
        void pose(const Eigen::Matrix4f &_pose);

        /// Get current estimated pose of the dataframe
        Eigen::Matrix4f pose() const;

        /// Set dense point cloud of the dataframe
        void cloud(const typename pcl::PointCloud<PointType_>::Ptr & _cloud);

        /// Get dense point cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr cloud() const;

        /// Set feature/sparse point cloud of the dataframe
        void featureCloud(const typename pcl::PointCloud<PointType_>::Ptr &_cloud);
        
        /// get feature/sparse point cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr featureCloud() const;

        /// Set feature descriptors associated to the dataframe
        void featureDescriptors(const cv::Mat &_descriptors);

        /// Get feature descriptors associated to the dataframe
        cv::Mat featureDescriptors() const;

        /// Set feature projections associated to the dataframe
        void featureProjections(const std::vector<cv::Point2f> &_projs);

        /// Get feature projections associated to the dataframe
        std::vector<cv::Point2f> featureProjections() const;

        /// Get if df has been optimized.
        bool isOptimized() const;

        /// Set df as optimized
        void isOptimized(const bool _opt);

        /// Get image associated to the left camera of the dataframe
        cv::Mat leftImage() const;

        /// Set image associated to the left camera of the dataframe
        void leftImage(const cv::Mat &_image);

        /// Get image associated to the right camera of the dataframe
        cv::Mat rightImage() const;

        /// Set image associated to the right camera of the dataframe
        void rightImage(const cv::Mat &_image);
        
        /// Get image associated to the depth camera of the dataframe
        cv::Mat depthImage() const;

        /// Set image associated to the depth camera of the dataframe
        void depthImage(const cv::Mat &_image);

        /// Get intrinsic coefficients of the camera associated to the dataframe
        cv::Mat intrinsics() const;

        /// Set intrinsic coefficients of the camera associated to the dataframe
        void intrinsics(const cv::Mat &_intrinsics);

        /// Get distorsion coefficients of the camera associated to the dataframe
        cv::Mat distCoeff() const;

        /// Set distorsion coefficients of the camera associated to the dataframe
        void distCoeff(const cv::Mat &_coeff);

        std::map<int, std::vector<cv::DMatch>> &crossReferencedInliers();

        #ifdef USE_DBOW2
            void signature(DBoW2::BowVector &_signature);
            DBoW2::BowVector signature() const;

            void featureVector(DBoW2::FeatureVector &_vector);
            DBoW2::FeatureVector featureVector() const;
        #endif


        /// Find and create words comparing current dataframe and the ones in covisibility. 
        /// This method should be called once.
        void wordCreation();    

        void reinforce(std::shared_ptr<Dataframe<PointType_>> &_df);
        
        friend std::ostream& operator<<(std::ostream& os, const Dataframe<PointType_>::Ptr _df){
            os << _df->id();
            return os;
        };
        
    private:
        Dataframe();    // Private void constructor to prevent its creation without ID.

        void reinforceWords(std::shared_ptr<mico::Dataframe<PointType_>> _df); 
    private:
        // ID of dataframe
        size_t id_;
        
        // Dense cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr cloud_;
        
        // Feature cloud of the dataframe and related information
        typename pcl::PointCloud<PointType_>::Ptr featureCloud_;
        cv::Mat                         featureDescriptors_;
        std::vector<cv::Point2f>        featureProjections_;

        // Cross reference of features in the other DFs
        std::map<int, std::vector<cv::DMatch>>         multimatchesInliersDfs_;
        std::map<int, std::shared_ptr<Word<PointType_>>>          wordsReference_;

        // Reference of which other df is observed from this one.
        std::vector<Dataframe<PointType_>::Ptr> covisibility_;

        // Visual information of the dataframe
        cv::Mat left_, right_, depth_;

        // Pose information 
        Eigen::Vector3f     position_;
        Eigen::Quaternionf  orientation_;
        Eigen::Matrix4f     pose_ = Eigen::Matrix4f::Identity();

        // Camera information
        cv::Mat intrinsics_;
        cv::Mat coefficients_;

        // Util flags
        bool optimized_ = false;

        // Signature of dataframe  666 Possible optimization with trait? so can use different implementations
        #ifdef USE_DBOW2
            DBoW2::BowVector signature_;
            DBoW2::FeatureVector featVec_;
        #endif
    };
}

#include <mico/base/map3d/Dataframe.inl>

#endif
