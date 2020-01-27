//---------------------------------------------------------------------------------------------------------------------
//  MICO
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

#ifndef MICO_BASE_MAP3D_ODOMETRYPHOTOGRAMMETRY_H_
#define MICO_BASE_MAP3D_ODOMETRYPHOTOGRAMMETRY_H_

#include <opencv2/opencv.hpp>

#include <mico/base/map3d/Dataframe.h>
#include <mico/base/StereoCamera.h>
#include <mico/base/utils/LogManager.h>
#include <mico/base/cjson/json.h>
#include <mico/base/map3d/Odometry.h>


namespace mico {

	// Set debug levels and out interface when instance object, not there.
  template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
  class OdometryPhotogrammetry : public Odometry<PointType_, DebugLevel_, OutInterface_>{
    public:
      /// Initializes RANSAC and ICP parameters
      virtual bool init(cjson::Json _configFile);
      
      /// Pick up an image from the camera and get a keyframe with the point cloud and feature cloud
      virtual bool computeOdometry(std::shared_ptr<mico::Dataframe<PointType_>> _prevDf, std::shared_ptr<mico::Dataframe<PointType_>> _currentDf);

      /// \brief Set factor param used to filter descriptors.
      /// \param _factor: factor.
      void descriptorDistanceFactor(double _factor);

      /// \brief Set number of iterations for the ransac process used for rought alignment and outlier filtering
      /// \param _iterations: confidence.
      void ransacIterations(int _iterations);

      /// \brief Set max distance allowed for considering inliers for the ransac process used for rought alignment and outlier filtering
      /// \param _maxDistance: maximum reprojection error.
      void ransacMaxDistance(double _maxDistance);

      /// \brief Set minimum number of inliers for considering the cloud for the ransac process used for rought alignment and outlier filtering
      /// \param _minInliers: maximum reprojection error.
      void ransacMinInliers(int _minInliers);

      /// \brief Set minimum number of inliers for considering the cloud for the ransac process used for rought alignment and outlier filtering
      /// \param _minInliers: maximum reprojection error.
      void ransacRefineIterations(int _refineIterations);

      /// \brief Set k nearest neighbors for matching descriptors
      /// \param _k_nearest_neighbors: Number of neighbors of a descriptor
      void k_nearest_neighbors(int _k_nearest_neighbors);

      /// \brief enable/disable icp
      /// \param _enable: true to enable false to disable
      void icpEnabled(int _icpEnabled);

       /// \brief Set maximum distance used for searching correspondences.
      /// \param _distance.
      void icpMaxCorrespondenceDistance(double _icpMaxCorrespondenceDistance);

      /// \brief Set maximum fitnes score for icp convergence
      /// \param _minInliers: maximum reprojection error.
      void icpMaxFitnessScore(double _icpMaxFitnessScore);

      /// \brief  Set maximum iterations for icp convergence
      /// \param _minInliers: maximum reprojection error.
      void icpMaxIterations(int _icpMaxIterations);

      /// \brief Set maximum transformation allowed for icp convergence criteria
      /// \param _maxExpsilon.
      void icpMaxTransformationEpsilon(double _icpMaxTransformationEpsilon);

      /// \brief Set distance for voxel filtering
      /// \param _distance.
      void icpVoxelDistance(double _icpVoxelDistance);

      /// \brief Set time out for ICP computing
      /// \param _distance.
      void icpTimeOut(double _icpTimeOut);

    private:
      /// Compute two keyframes to get his transform and matches
      bool compute(std::shared_ptr<mico::Dataframe<PointType_>> _prevDf, std::shared_ptr<mico::Dataframe<PointType_>> _currentDf);
					
    private:
      /// RANSAC parameters
      double   mFactorDescriptorDistance = 35.0;
      double   mK_nearest_neighbors = 1;
      int      mRansacIterations = 50;
      double   mRansacMaxDistance = 5.0;
      int      mRansacMinInliers = 20;
      unsigned mRansacRefineIterations = 0;

      /// ICP parameters
      int    mIcpEnabled = 1; 
      double mIcpMaxCorrespondenceDistance = 1.0;
      double mIcpMaxFitnessScore = 20.0;
      int    mIcpMaxIterations = 15;
      double mIcpMaxTransformationEpsilon = 0.00001;
      double mIcpVoxelDistance =0.1;
      double mIcpTimeOut = 5000.0;
  };
} // namespace mico

#include <mico/base/map3d/OdometryPhotogrammetry.inl>

#endif // ODOMETRYPHOTOGRAMETRY_H_
