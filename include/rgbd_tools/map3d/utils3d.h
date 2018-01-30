////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDTOOLS_MAP3D_UTILS3D_H_
#define RGBDTOOLS_MAP3D_UTILS3D_H_

#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rgbd{

    template<typename PointType_>
    /// Perform alignment between two clouds using RANSAC
    /// \param _source:
    /// \param _target:
    /// \param _matches:
    /// \param _transformation:
    /// \param _inliers:
    /// \param _maxRansacDistance:
    /// \param _ransacIterations:
    /// \param _refineIterations:
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
    template<typename PointType_>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations = 10,
                      double _correspondenceDistance = 0.3,
                      double _maxAngleDistance = 0.707,
                      double _maxColorDistance = 0.3,
                      double _maxTranslation = 0.01,
                      double _maxRotation = 0.01,
                      double _maxFitnessScore = 1.0
                      );

}

#include "utils3d.inl"

#endif