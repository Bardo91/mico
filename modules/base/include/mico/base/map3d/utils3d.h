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

#ifndef MICO_BASE_MAP3D_UTILS3D_H_
#define MICO_BASE_MAP3D_UTILS3D_H_

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include <mico/base/map3d/utils2d.h>
#include <mico/base/map3d/Dataframe.h>

#include <mico/base/utils/LogManager.h>

namespace mico {

    /// Perform alignment between two clouds using RANSAC
    /// \param _source:
    /// \param _target:
    /// \param _matches:
    /// \param _transformation:
    /// \param _inliers:
    /// \param _maxRansacDistance:
    /// \param _ransacIterations:
    /// \param _refineIterations:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance = 0.01,
                         int _ransacIterations = 3000,
                         unsigned _refineIterations = 5);

    /// Perform alignement between two clouds given an initial transformation
    /// \param _source:
    /// \param _target:
    /// \param _transformation:
    /// \param _correspondenceDistance:
    /// \param _maxAngleDistance:
    /// \param _maxColorDistance:
    /// \param _maxTranslation:
    /// \param _maxRotation:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations = 10,
                      double _correspondenceDistance = 0.3,
                      double _maxAngleDistance = 0.707,
                      double _maxColorDistance = 0.3,
                      double _maxTranslation = 0.01,
                      double _maxRotation = 0.01,
                      double _maxFitnessScore = 1.0,
                      double _voxelGridSize = 0.03,
                      double _timeout = std::numeric_limits<double>::max());

    /// Perform alignement between two clouds given an initial transformation
    /// \param _source:
    /// \param _target:
    /// \param _transformation:
    /// \param _correspondenceDistance:
    /// \param _maxAngleDistance:
    /// \param _maxColorDistance:
    /// \param _maxTranslation:
    /// \param _maxRotation:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool icpPhotogrammetry(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations = 10,
                      double _correspondenceDistance = 0.3,
                      double _maxAngleDistance = 0.707,
                      double _maxColorDistance = 0.3,
                      double _maxTranslation = 0.01,
                      double _maxRotation = 0.01,
                      double _maxFitnessScore = 1.0,
                      double _voxelGridSize = 0.03,
                      double _timeout = std::numeric_limits<double>::max());
                      
    /// Compute roughtly but robustly the transformation between given keyframes.
    /// \param _source:
    /// \param _target:
    /// \param _transformation:
    /// \param _iterations:
    /// \param _correspondenceDistance:
    /// \param _maxAngleDistance:
    /// \param _maxColorDistance:
    /// \param _maxTranslation:
    /// \param _maxRotation:
    /// \param _maxFitnessScore:
    /// \param timeout:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool transformationBetweenFeatures(std::shared_ptr<Dataframe<PointType_>> &_previousKf,
                                       std::shared_ptr<Dataframe<PointType_>> &_currentKf,
                                       Eigen::Matrix4f &_transformation,
                                       double _mk_nearest_neighbors,
                                       double _mRansacMaxDistance,
                                       int _mRansacIterations,
                                       double _mRansacMinInliers,
                                       double _mFactorDescriptorDistance,
                                       unsigned _mRansacRefineIterations=5);
/// Compute principal component analysis.
    /// \param _cloud:
    /// \param _pose:
    /// \param _limits:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool computePCA(typename pcl::PointCloud<PointType_> &_cloud,
                    Eigen::Matrix4f &_pose,
                    std::vector<float> &_limits);

}

#include "utils3d.inl"

#endif
