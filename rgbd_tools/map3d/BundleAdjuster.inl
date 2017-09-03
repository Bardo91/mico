////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifdef USE_CVSBA
    #include <cvsba/cvsba.h>
#endif

#include <unordered_map>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::optimize() {
        #ifdef USE_CVSBA
            prepareData();

            std::vector<cv::Mat> ts, rs, intrinsics, coeffs;
            for(auto kf: mKeyframes){
                intrinsics.push_back(kf.intrinsic);
                coeffs.push_back(kf.coefficients);
                ts.push_back(cv::Mat(3,1,CV_32F, &kf.position[0]));
                auto rotation = kf.orientation.matrix();
                auto cvRotation = cv::Mat(3,3,CV_32F, rotation.data());
                cv::Rodrigues(cvRotation, cvRotation);
                rs.push_back(cvRotation);
            }

            // Initialize cvSBA and perform bundle adjustment.
            cvsba::Sba bundleAdjuster;
            cvsba::Sba::Params params;
            params.verbose = true;
            params.iterations = mBaIterations;
            params.minError = mBaMinError;
            bundleAdjuster.setParams(params);

            assert(mScenePoints.size() == mScenePointsProjection[0].size());   // 666 TODO create check method.
            assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
            assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
            assert(intrinsics.size() == mScenePointsProjection.size());
            assert(intrinsics.size() == coeffs.size());
            assert(intrinsics.size() == rs.size());
            assert(ts.size() == rs.size());


            // I think it is no necessary, it is SPARSE BA.
            // count aparitions
            std::vector<int> aparitions(mCovisibilityMatrix[0].size());
            for(unsigned i = 0; i < mCovisibilityMatrix.size(); i++){
                for(unsigned j = 0; j < mCovisibilityMatrix[0].size(); j++){
                    aparitions[j] += mCovisibilityMatrix[i][j];
                }
            }
            // Create data only for elementes with enough aparitions. 666 TODO improve mem alloc! possible bottle neck
             std::vector<std::vector<int>> visibility(mCovisibilityMatrix.size());
             std::vector<cv::Point3f> points;
             std::vector<std::vector<cv::Point2f>> projections(mCovisibilityMatrix.size());
             for(unsigned i = 0; i < aparitions.size(); i++){
                 if(aparitions[i] > mBaMinAparitions){
                     points.push_back(mScenePoints[i]);
                     for(unsigned j = 0; j < mCovisibilityMatrix.size(); j++){
                         projections[j].push_back(mScenePointsProjection[j][i]);
                         visibility[j].push_back(mCovisibilityMatrix[j][i]);
                     }
                 }
             }

            bundleAdjuster.run(points, projections, visibility, intrinsics, rs, ts, coeffs);

            for(unsigned i = 0; i < ts.size(); i++){
               Eigen::Affine3f pose;
               cv::Mat R;
               cv::Rodrigues(rs[i], R);

               Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

               rotation(0,0) = R.at<double>(0,0);
               rotation(0,1) = R.at<double>(0,1);
               rotation(0,2) = R.at<double>(0,2);
               rotation(1,0) = R.at<double>(1,0);
               rotation(1,1) = R.at<double>(1,1);
               rotation(1,2) = R.at<double>(1,2);
               rotation(2,0) = R.at<double>(2,0);
               rotation(2,1) = R.at<double>(2,1);
               rotation(2,2) = R.at<double>(2,2);
               rotation(0,3) = ts[i].at<double>(0);
               rotation(1,3) = ts[i].at<double>(1);
               rotation(2,3) = ts[i].at<double>(2);

               pose.matrix() = rotation;

               //viewer.addCoordinateSystem(0.15, pose, "camera_" + std::to_string(i));

               mKeyframes[i].position = rotation.block<3,1>(0,3);
               mKeyframes[i].orientation = rotation.block<3,3>(0,0).matrix();
               mKeyframes[i].pose = rotation;
            }

            //
            return true;
        #else
            std::cout << "CVSBA not installed! CANT PERFORM BA" << std::endl;
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::keyframes(std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>> > &_keyframes) {
        mKeyframes = _keyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::keyframes(typename std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>> >::iterator &_begin, typename std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>> >::iterator &_end) {
        mKeyframes.erase();
        mKeyframes.insert(mKeyframes.begin(), _begin, _end);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double BundleAdjuster<PointType_>::minError       () const{
        return mBaMinError;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned BundleAdjuster<PointType_>::iterations     () const{
        return mBaIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned BundleAdjuster<PointType_>::minAparitions  () const{
        return mBaMinAparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::minError         (double _error){
        mBaMinError = _error;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::iterations       (unsigned _iterations){
        mBaIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::minAparitions    (unsigned _aparitions){
        mBaMinAparitions = _aparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    std::vector<Keyframe<PointType_> , Eigen::aligned_allocator <Keyframe<PointType_>>> BundleAdjuster<PointType_>::keyframes() {
        return mKeyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void BundleAdjuster<PointType_>::cleanData() {
        mScenePoints.clear();
        mScenePointsProjection.clear();
        mScenePointsProjection.resize(mKeyframes.size());
        mCovisibilityMatrix.clear();
        mCovisibilityMatrix.resize(mKeyframes.size());
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::prepareData() {
        cleanData();

        std::unordered_map<int, int> idToIdx; // Map that maps from world id to data idx;
        // Allocate covisibility matrix for max size and then reduce at the end
        int maxSizeMat = 0;
        for(unsigned i = 0; i < mKeyframes.size(); i++){maxSizeMat += mKeyframes[i].featureProjections.size();};


        int lastIdx = 0;
        for(unsigned kfIdx = 0; kfIdx < mKeyframes.size(); kfIdx++){
            mCovisibilityMatrix[kfIdx].resize(maxSizeMat, 0);
            for(unsigned wIdx = 0; wIdx < mKeyframes[kfIdx].wordsReference.size(); wIdx++){
                auto &w = mKeyframes[kfIdx].wordsReference[wIdx];
                auto idIter = idToIdx.find(w->id);
                if(idIter != idToIdx.end()){ // If word already added.
                    int id = idToIdx[w->id];
                    mCovisibilityMatrix[kfIdx][id] = 1;
                    mScenePointsProjection[kfIdx][id] = mKeyframes[kfIdx].featureProjections[wIdx]; // Check that all feature points are added as words.
                }else{  // If word not added yet
                    int id = lastIdx;
                    lastIdx++;
                    int wId = w->id;
                    idToIdx[wId] = id;
                    mCovisibilityMatrix[kfIdx][id] = 1;
                    mScenePoints.push_back(cv::Point3f(w->point[0],w->point[1],w->point[2]));
                    for(auto &v: mScenePointsProjection){
                        v.push_back(cv::Point2f(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
                    }
                    mScenePointsProjection[kfIdx][id] = mKeyframes[kfIdx].featureProjections[wIdx];
                }
            }
        }
        for(auto &v: mCovisibilityMatrix){
            v.resize(idToIdx.size());
        }
    }

}