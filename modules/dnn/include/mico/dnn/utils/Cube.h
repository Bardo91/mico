//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Ricardo Lopez Lopez (a.k.a Ric92)
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

#ifndef MICO_DNN_UTILS_CUBE_H_
#define MICO_DNN_UTILS_CUBE_H_

#include <Eigen/Eigen>
#include <utility>
#include <unordered_map>

#include <mico/dnn/utils/Facet.h>
#include <mico/dnn/utils/ConvexPolyhedron.h>
namespace mico {
class Cube : public ConvexPolyhedron {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // _pose: Pose of Frustum
    // _hfov,_vfov: Vertical and horizontal FOV in degrees
    // _npDistance,fpDistance: Distance between pose and nearest and farest
    // plane
    Cube(int _id, Eigen::Matrix4f _pose, float _width, float _heigth,
         float _deep) {
        id = _id;
        mPose = _pose;
        mWidth = _width;
        mHeigth = _heigth;
        mDeep = _deep;

        // back plane
        // heigth
        // ^
        // |
        // |-->width
        //
        // v4----v1
        // |      |
        // v3----v2
        Eigen::Vector3f v1(mWidth / 2, mDeep / 2, mHeigth / 2);
        Eigen::Vector3f v2(mWidth / 2, mDeep / 2, -mHeigth / 2);
        Eigen::Vector3f v3(-mWidth / 2, mDeep / 2, -mHeigth / 2);
        Eigen::Vector3f v4(-mWidth / 2, mDeep / 2, mHeigth / 2);
        // front plane
        // v8----v5
        // |      |
        // v7----v6
        Eigen::Vector3f v5(mWidth / 2, -mDeep / 2, mHeigth / 2);
        Eigen::Vector3f v6(mWidth / 2, -mDeep / 2, -mHeigth / 2);
        Eigen::Vector3f v7(-mWidth / 2, -mDeep / 2, -mHeigth / 2);
        Eigen::Vector3f v8(-mWidth / 2, -mDeep / 2, mHeigth / 2);
		
        v1 = mPose.block(0, 0, 3, 3) * v1 + mPose.block(0, 3, 3, 1);
        v2 = mPose.block(0, 0, 3, 3) * v2 + mPose.block(0, 3, 3, 1);
        v3 = mPose.block(0, 0, 3, 3) * v3 + mPose.block(0, 3, 3, 1);
        v4 = mPose.block(0, 0, 3, 3) * v4 + mPose.block(0, 3, 3, 1);
        v5 = mPose.block(0, 0, 3, 3) * v5 + mPose.block(0, 3, 3, 1);
        v6 = mPose.block(0, 0, 3, 3) * v6 + mPose.block(0, 3, 3, 1);
        v7 = mPose.block(0, 0, 3, 3) * v7 + mPose.block(0, 3, 3, 1);
        v8 = mPose.block(0, 0, 3, 3) * v8 + mPose.block(0, 3, 3, 1);

        std::vector<Eigen::Vector3f> cubeVertices;
        cubeVertices = {v1, v2, v3, v4, v5, v6, v7, v8};
        //                  TOP      BACK
        //                   |      /
        //               v4----v1  /
        //              /     /
        //             /     /
        // LEFT<--    v8----v5     --> RIGHT
        //               v3----v2
        //              /     /
        //             /     /
        //            v7----v6
        //           /
        //          /
        //      FRONT
        std::unordered_map<std::string, std::shared_ptr<Facet>> cubeFacets;

        // Plane eq: Ax + By + Cz + D = 0
        // Back plane
        mBackplaneNormal = (v1 - v2).cross(v2 - v3);
        mBackplaneNormal.normalize();
        mBackplane.head(3) = mBackplaneNormal;
        mBackplane[3] = -v1.dot(mBackplaneNormal);
        std::vector<Eigen::Vector3f> backVertex = {v1, v2, v3, v4};
        std::shared_ptr<Facet> backFacet(new Facet(mBackplane, backVertex));
        cubeFacets["back"] = backFacet;

        // Front plane
        mFrontplaneNormal = (v5 - v6).cross(v7 - v6);
        mFrontplaneNormal.normalize();
        mFrontplane.head(3) = mFrontplaneNormal;
        mFrontplane[3] = -v5.dot(mFrontplaneNormal);
        std::vector<Eigen::Vector3f> nearVertex = {v5, v6, v7, v8};
        std::shared_ptr<Facet> frontFacet(new Facet(mFrontplane, nearVertex));
        cubeFacets["front"] = frontFacet;

        // Top plane
        mTopPlaneNormal = (v1 - v5).cross(v4 - v1);
        mTopPlane.head(3) = mTopPlaneNormal;
        mTopPlane[3] = -v1.dot(mTopPlaneNormal);
        std::vector<Eigen::Vector3f> topVertex = {v1, v5, v8, v4};
        std::shared_ptr<Facet> topFacet(new Facet(mTopPlane, topVertex));
        cubeFacets["top"] = topFacet;

        // Down plane
        mDownPlaneNormal = (v2 - v6).cross(v6 - v7);
        mDownPlaneNormal.normalize();
        mDownPlane.head(3) = mDownPlaneNormal;
        mDownPlane[3] = -v2.dot(mDownPlaneNormal);
        std::vector<Eigen::Vector3f> downVertex = {v2, v6, v7, v3};
        std::shared_ptr<Facet> downFacet(new Facet(mDownPlane, downVertex));
        cubeFacets["down"] = downFacet;

        // Right plane
        mRightPlaneNormal = (v1 - v2).cross(v6 - v2);
        mRightPlaneNormal.normalize();
        mRightPlane.head(3) = mRightPlaneNormal;
        mRightPlane[3] = -v1.dot(mRightPlaneNormal);
        std::vector<Eigen::Vector3f> rightVertex = {v1, v5, v6, v2};
        std::shared_ptr<Facet> rightFacet(new Facet(mRightPlane, rightVertex));
        cubeFacets["right"] = rightFacet;

        // Left plane
        mLeftPlaneNormal = (v8 - v7).cross(v3 - v7);
        mLeftPlaneNormal.normalize();
        mLeftPlane.head(3) = mLeftPlaneNormal;
        mLeftPlane[3] = -v8.dot(mLeftPlaneNormal);
        std::vector<Eigen::Vector3f> leftVertex = {v4, v8, v7, v3};
        std::shared_ptr<Facet> leftFacet(new Facet(mLeftPlane, leftVertex));
        cubeFacets["left"] = leftFacet;

        setFacets(cubeFacets);
        setVertices(cubeVertices);
        setVolume(mWidth * mHeigth * mDeep);
    }

    int id;

    // Frustum pose
    Eigen::Matrix4f mPose;

    float mWidth, mHeigth, mDeep;

    // Back plane
    Eigen::Vector4f mBackplane;
    Eigen::Vector3f mBackplaneNormal;

    // Front plane
    Eigen::Vector4f mFrontplane;
    Eigen::Vector3f mFrontplaneNormal;

    // Top plane
    Eigen::Vector4f mTopPlane;
    Eigen::Vector3f mTopPlaneNormal;

    // Down plane
    Eigen::Vector4f mDownPlane;
    Eigen::Vector3f mDownPlaneNormal;

    // Right plane
    Eigen::Vector4f mRightPlane;
    Eigen::Vector3f mRightPlaneNormal;

    // Left plane
    Eigen::Vector4f mLeftPlane;
    Eigen::Vector3f mLeftPlaneNormal;
};
}

#endif // MICO_DNN_UTILS_CUBE_H_