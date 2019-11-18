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


#include <Eigen/Eigen>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <chrono>

namespace mico {

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance,
                         int _ransacIterations,
                         unsigned _refineIterations) {

        std::vector<int> source_indices (_matches.size());   // 666 matchesPrev should be removed from kfs struct.
        std::vector<int> target_indices (_matches.size());

        // Copy the query-match indices
        for (int i = 0; i < (int)_matches.size(); ++i) {
            source_indices[i] = _matches[i].queryIdx;
            target_indices[i] = _matches[i].trainIdx;
        }

        typename pcl::SampleConsensusModelRegistration<PointType_>::Ptr model(new pcl::SampleConsensusModelRegistration<PointType_>(_source, source_indices));

        // Pass the target_indices
        model->setInputTarget (_target, target_indices);

        // Create a RANSAC model
        pcl::RandomSampleConsensus<PointType_> sac (model, _maxRansacDistance);

        sac.setMaxIterations(_ransacIterations);
        // Compute the set of inliers
        if(sac.computeModel()) {
            Eigen::VectorXf model_coefficients;

            sac.getInliers(_inliers);

            sac.getModelCoefficients (model_coefficients);

            // REFINEMENT STEP
            if (_refineIterations > 0) {
                double error_threshold = _maxRansacDistance;
                unsigned int refine_iterations = 0;
                bool inlier_changed = false;
                  bool oscillating = false;
                std::vector<int> new_inliers, prev_inliers = _inliers;
                std::vector<size_t> inliers_sizes;
                Eigen::VectorXf new_model_coefficients = model_coefficients;
                do {
                    // Optimize the model coefficients
                    model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                    inliers_sizes.push_back (prev_inliers.size ());

                    // Select the new inliers based on the optimized coefficients and new threshold
                    model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);

                    if (new_inliers.empty ()) {
                        ++refine_iterations;
                        if (refine_iterations >= _refineIterations) {
                            break;
                        }
                        continue;
                    }

                    // Estimate the variance and the new threshold
                    double variance = model->computeVariance ();
                    double refineSigma = 3.0;
                    error_threshold = std::min (_maxRansacDistance, refineSigma * sqrt(variance));

                    inlier_changed = false;
                    std::swap (prev_inliers, new_inliers);

                    // If the number of inliers changed, then we are still optimizing
                    if (new_inliers.size () != prev_inliers.size ()) {
                        // Check if the number of inliers is oscillating in between two values
                        if (inliers_sizes.size () >= 4) {
                            if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4]) {
                                oscillating = true;
                                break;
                            }
                        }
                        inlier_changed = true;
                        continue;
                    }

                    // Check the values of the inlier set
                    for (size_t i = 0; i < prev_inliers.size (); ++i) {
                        // If the value of the inliers changed, then we are still optimizing
                        if (prev_inliers[i] != new_inliers[i]){
                            inlier_changed = true;
                            break;
                        }
                    }
                } while (inlier_changed && ++refine_iterations < _refineIterations);

                std::swap (_inliers, new_inliers);
                model_coefficients = new_model_coefficients;
            }


              double covariance = model->computeVariance();
            // get best transformation
            Eigen::Matrix4f bestTransformation;
            bestTransformation.row (0) = model_coefficients.segment<4>(0);
            bestTransformation.row (1) = model_coefficients.segment<4>(4);
            bestTransformation.row (2) = model_coefficients.segment<4>(8);
            bestTransformation.row (3) = model_coefficients.segment<4>(12);
            _transformation = bestTransformation;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations,
                      double _correspondenceDistance,
                      double _maxAngleDistance,
                      double _maxColorDistance,
                      double _maxTranslation,
                      double _maxRotation,
                      double _maxFitnessScore,
                      double _voxelGridSize,
                      double _timeout) {
        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        auto t0 = std::chrono::high_resolution_clock::now();
        double timeSpent = 0;
        pcl::PointCloud<PointType_> srcCloud;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_source, srcCloud, indices);
        pcl::PointCloud<PointType_> tgtCloud;
        pcl::removeNaNFromPointCloud(*_target, tgtCloud, indices);

        pcl::VoxelGrid<PointType_> sor;
        sor.setLeafSize (_voxelGridSize,_voxelGridSize,_voxelGridSize);
        sor.setInputCloud (srcCloud.makeShared());
        sor.filter (srcCloud);
        sor.setInputCloud (tgtCloud.makeShared());
        sor.filter (tgtCloud);

        bool converged = false;
        int iters = 0;
        double corrDistance = _correspondenceDistance;
        while (/*!converged &&*/ iters < _iterations && timeSpent < _timeout) {   
            iters++;
            pcl::PointCloud<PointType_> cloudToAlign;
            //std::cout << _transformation << std::endl;
            pcl::transformPointCloudWithNormals(srcCloud, cloudToAlign, _transformation);
            
            // COMPUTE CORRESPONDENCES
            pcl::Correspondences correspondences;
            pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences);

            pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
            corresp_kdtree.setInputSource(cloudToAlign.makeShared());
            corresp_kdtree.setInputTarget(tgtCloud.makeShared());
            corresp_kdtree.determineCorrespondences(*ptrCorr, corrDistance);

            //std::cout << "Found " << ptrCorr->size() << " correspondences by distance" << std::endl;

            if (ptrCorr->size() == 0) {
                logDealer.error("ICP_ALIGNEMENT", "Can't find any correspondence");
                //std::cout << "Can't find any correspondences!" << std::endl;
                break;
            }
            else {
                pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejectorNormal(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
                rejectorNormal->setThreshold(_maxAngleDistance);
                rejectorNormal->initializeDataContainer<PointType_, PointType_>();
                rejectorNormal->setInputSource<PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputNormals<PointType_, PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputTarget<PointType_>(tgtCloud.makeShared());
                rejectorNormal->setTargetNormals<PointType_, PointType_>(tgtCloud.makeShared());
                rejectorNormal->setInputCorrespondences(ptrCorr);
                rejectorNormal->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after normal rejection" << std::endl;

                pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);
                rejector->setInputCorrespondences(ptrCorr);
                rejector->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after one to one rejection" << std::endl;

                //pcl::CorrespondencesPtr ptrCorr2(new pcl::Correspondences);
                if(fabs(_maxColorDistance - 1)  < 0.01){    // 666 Just in case
                    // Reject by color
                    for (auto corr : *ptrCorr) {
                        // Measure distance
                        auto p1 = cloudToAlign.at(corr.index_query);
                        auto p2 = tgtCloud.at(corr.index_match);
                        double dist = sqrt(pow(p1.r - p2.r, 2) + pow(p1.g - p2.g, 2) + pow(p1.b - p2.b, 2));
                        dist /= sqrt(3) * 255;

                        // Add if approved
                        if (dist < _maxColorDistance) {
                            correspondences.push_back(corr);
                        }
                    }
                }else{
                    correspondences = *ptrCorr;
                }

                //std::cout << "Found " << correspondences.size() << " correspondences after color rejection" << std::endl;
            }

            // Estimate transform
            pcl::registration::TransformationEstimationPointToPlaneLLS<PointType_, PointType_, float> estimator;
            //std::cout << _transformation << std::endl;
            Eigen::Matrix4f incTransform;
            estimator.estimateRigidTransformation(cloudToAlign, tgtCloud, correspondences,  incTransform);
            if (incTransform.hasNaN()) {
                logDealer.error("ICP_ALIGNEMENT", "Transformation of the cloud contains NaN");
                continue;
            }

            // COMPUTE SCORE
            double score = 0;
            for (unsigned j = 0; j < correspondences.size(); j++) {
                score += correspondences[j].distance;
            }

            // CONVERGENCE
            Eigen::Matrix3f rot = incTransform.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rot);
            Eigen::Quaternionf q0(Eigen::Matrix3f::Identity());
            double rotRes = fabs(q0.x() - q.x())+fabs(q0.z() - q.z())+fabs(q0.y() - q.y())+fabs(q0.w() - q.w());
            double transRes = fabs(incTransform.block<3, 1>(0, 3).sum());
            converged = (rotRes < _maxRotation &&  transRes < _maxTranslation) ? 1 : 0;

            //std::cout << "incT: " << transRes << ". incR: " << rotRes << ". Score: " << score << std::endl;
            converged = converged && (score < _maxFitnessScore);
            _transformation = incTransform*_transformation;
            corrDistance *= 0.9;     

            auto t1 = std::chrono::high_resolution_clock::now();
            timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        }

        return converged;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool icpPhotogrammetry(typename pcl::PointCloud<PointType_>::Ptr _source,
                           typename pcl::PointCloud<PointType_>::Ptr _target,
                           Eigen::Matrix4f &_transformation,
                           int _iterations,
                           double _correspondenceDistance,
                           double _maxAngleDistance,
                           double _maxColorDistance,
                           double _maxTranslation,
                           double _maxRotation,
                           double _maxFitnessScore,
                           double _voxelGridSize,
                           double _timeout) {
        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        auto t0 = std::chrono::high_resolution_clock::now();
        double timeSpent = 0;
        typename pcl::PointCloud<PointType_>::Ptr srcCloud(new pcl::PointCloud<PointType_>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_source, *srcCloud, indices);
        typename pcl::PointCloud<PointType_>::Ptr tgtCloud(new pcl::PointCloud<PointType_>);
        pcl::removeNaNFromPointCloud(*_target, *tgtCloud, indices);

        bool converged = false;
        int iters = 0;
        double corrDistance = _correspondenceDistance;
        while (/*!converged &&*/ iters < _iterations && timeSpent < _timeout) {   
            iters++;
            typename pcl::PointCloud<PointType_>::Ptr cloudToAlign(new pcl::PointCloud<PointType_>);
            pcl::transformPointCloud(*srcCloud, *cloudToAlign, _transformation);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloudToAlign, *cloudToAlign, indices);            
            
            pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences);

            // rejector by distance
            pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
            corresp_kdtree.setInputSource(cloudToAlign);
            corresp_kdtree.setInputTarget(tgtCloud);
            corresp_kdtree.determineCorrespondences(*ptrCorr, corrDistance);

            //std::cout << "Found " << ptrCorr->size() << " correspondences by distance" << std::endl;

            if (ptrCorr->size() == 0) {
                logDealer.error("ICP_ALIGNEMENT", "Can't find any correspondence");
                //std::cout << "Can't find any correspondences!" << std::endl;
                break;
            }
            else {

                pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);
                rejector->setInputCorrespondences(ptrCorr);
                rejector->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after one to one rejection" << std::endl;
            }

            // Estimate transform
            Eigen::Matrix4f incTransform;
            pcl::registration::TransformationEstimationSVD<PointType_, PointType_, float> estimator;
            estimator.estimateRigidTransformation(*cloudToAlign, *tgtCloud, *ptrCorr,  incTransform);
            if (incTransform.hasNaN()) {
                logDealer.error("ICP_ALIGNEMENT", "Transformation of the cloud contains NaN");
                continue;
            }

            // COMPUTE SCORE
            double score = 0;
            for (unsigned j = 0; j < (*ptrCorr).size(); j++) {
                score += (*ptrCorr)[j].distance;
            }

            // CONVERGENCE
            Eigen::Matrix3f rot = incTransform.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rot);
            Eigen::Quaternionf q0(Eigen::Matrix3f::Identity());
            double rotRes = fabs(q0.x() - q.x())+fabs(q0.z() - q.z())+fabs(q0.y() - q.y())+fabs(q0.w() - q.w());
            double transRes = fabs(incTransform.block<3, 1>(0, 3).sum());
            converged = (rotRes < _maxRotation &&  transRes < _maxTranslation) ? 1 : 0;

            //std::cout << "incT: " << transRes << ". incR: " << rotRes << ". Score: " << score << std::endl;
            converged = converged && (score < _maxFitnessScore);
            _transformation = incTransform*_transformation;
            corrDistance *= 0.9;     

            auto t1 = std::chrono::high_resolution_clock::now();
            timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        }

        return converged;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool  transformationBetweenFeatures(std::shared_ptr<Dataframe<PointType_>> &_previousDf,
                                        std::shared_ptr<Dataframe<PointType_>> &_currentDf,
                                        Eigen::Matrix4f &_transformation,
                                        double _mk_nearest_neighbors,
                                        double _mRansacMaxDistance,
                                        int _mRansacIterations,
                                        double _mRansacMinInliers,
                                        double _mFactorDescriptorDistance,
                                        unsigned _mRansacRefineIterations){

        LoggableInterface<DebugLevel_, OutInterface_> logDealer;
        
        if(_currentDf->crossReferencedInliers().find(_previousDf->id()) !=  _currentDf->crossReferencedInliers().end()){
            // Match already computed
            logDealer.status("TRANSFORM_BETWEEN_FEATURES",  "Match already computed between frames: " + 
                                                            std::to_string(_currentDf->id()) + " and " + 
                                                            std::to_string(_previousDf->id()));
            return true;
        }
        std::vector<cv::DMatch> matches;
        if (!matchDescriptorsBF(_currentDf->featureDescriptors(),
                                _previousDf->featureDescriptors(),
                                matches,
                                _mk_nearest_neighbors,
                                _mFactorDescriptorDistance))
        {
            return false;
        }        
        std::vector<int> inliers;
        if(_mk_nearest_neighbors>1){
            typename pcl::PointCloud<PointType_>::Ptr duplicateCurrentKfFeatureCloud = _currentDf->featureCloud();
            *duplicateCurrentKfFeatureCloud += *_currentDf->featureCloud();
             mico::ransacAlignment<PointType_>( duplicateCurrentKfFeatureCloud,
                                                _previousDf->featureCloud(),
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }else {
            mico::ransacAlignment<PointType_>(  _currentDf->featureCloud(),
                                                _previousDf->featureCloud(),
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }
        // cv::Mat display = _currentDf->left.clone();
        // for(auto &match:matches){
        //     cv::Point p2 = _currentDf->featureProjections[match.queryIdx];
        //     cv::circle(display, p2, 3, cv::Scalar(0,255,0), 1);
        // }
        // std::cout << matches.size() << std::endl;
        // cv::imshow("FeatureMatcherRansac", display);
        // cv::waitKey();

        logDealer.status("TRANSFORM_BETWEEN_FEATURES", "Inliers between df " + std::to_string(_previousDf->id()) + " and kf " + 
                                                        std::to_string(_currentDf->id()) + " = " + std::to_string(inliers.size()));
        if (inliers.size() >= _mRansacMinInliers) {
            _currentDf->crossReferencedInliers()[_previousDf->id()];
            _previousDf->crossReferencedInliers()[_currentDf->id()];
            int j = 0;
            for(unsigned int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _currentDf->crossReferencedInliers()[_previousDf->id()].push_back(matches[j]);
                _previousDf->crossReferencedInliers()[_currentDf->id()].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

            }
            return true;
        }else{
            logDealer.error("TRANSFORM_BETWEEN_FEATURES", "Rejecting frame: Num Inliers <" + std::to_string(_mRansacMinInliers));
            return false;
        }
    }

    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool computePCA(typename pcl::PointCloud<PointType_> &_cloud,
                    Eigen::Matrix4f &_pose,
                    std::vector<float> &_limits){

        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid<PointType_>(_cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized<PointType_>(_cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
        typename pcl::PointCloud<PointType_>::Ptr cloudPointsProjected(new pcl::PointCloud<PointType_>);
        pcl::transformPointCloud(_cloud, *cloudPointsProjected, projectionTransform);

        // Get the minimum and maximum points of the transformed cloud.
        PointType_ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        _limits = {maxPoint.x, minPoint.x, maxPoint.y, minPoint.y, maxPoint.z, minPoint.z};

        const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        if(std::isnan(bboxTransform(0)) || std::isnan(bboxTransform(1)) || std::isnan(bboxTransform(2)) ){
            logDealer.error("COMPUTE_PRINCIPAL_COMPONENT_ANALYSIS", "NaN pose");
            return false; 
        }

        _pose.block(0,0,3,3) = bboxQuaternion.toRotationMatrix();
        _pose.block(0,3,3,1) = bboxTransform;
        return true;
    }
}

