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

#ifndef RGBD_MAP3D_WORD_H_
#define RGBD_MAP3D_WORD_H_

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <Eigen/Eigen>
#include <math.h>
#include <stdlib.h>


namespace mico {
    template<typename PointType_>
    class Dataframe;

    template <typename PointType_>
    class Word{
    public:
        typedef std::shared_ptr<Word<PointType_>> Ptr;

        Word(int _wordId, std::vector<float> _point3D, cv::Mat _descriptor);

        void addObservation(    std::shared_ptr<Dataframe<PointType_>> _df,
                                int _idx,
                                std::vector<float> _projections);

        bool isInFrame(int _id){
            return dfMap.find(_id) != dfMap.end();
        }

        cv::Point2f cvProjectionf(int _id){
            return cv::Point2f(projections[_id][0], projections[_id][1]);
        }

        cv::Point2d cvProjectiond(int _id){
            return cv::Point2d(projections[_id][0], projections[_id][1]);
        }

        cv::Point3f cvPointf(){
            return cv::Point3f(point[0], point[1], point[2]);
        }

        cv::Point3d cvPointd(){
            return cv::Point3d(point[0], point[1], point[2]);
        }

        PointType_ pclPoint(){
            PointType_ p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            return p;
        }

        void mergeWord(std::shared_ptr<Word<PointType_>> _word);

        bool eraseProjection(int _dfId);

        void updateNormal();

        void checkProjections();

        int id;
        std::vector<float> point;
        std::unordered_map<int, std::vector<float>> projections;
        std::unordered_map<int, bool> projectionsEnabled;
        std::unordered_map<int,std::unordered_map<int, cv::Mat>> descriptors; 
        
        std::unordered_map<int, int> idxInDf; // Related to features and projections
        cv::Mat descriptor;
        Eigen::Vector3f normalVector;

        std::map<int, std::shared_ptr<Dataframe<PointType_>>> dfMap; // TODO : Refactoring 
        

        bool optimized=false;

        // Getters
        PointType_ asPclPoint()
        {
            PointType_ pclPoint;
            pclPoint.x = point[0];
            pclPoint.y = point[1];
            pclPoint.z = point[2];
            return pclPoint;
        }
    };

    template <typename PointType_>
    std::ostream &operator<<(std::ostream &os, const Word<PointType_> &w){
        for (auto &proj : w.projections){
                os << proj.first << "," << proj.second[0] << "," << proj.second[1] << ",";
        }
        return os;
    }

} // namespace mico 

#include <mico/base/map3d/Word.inl>


#endif
