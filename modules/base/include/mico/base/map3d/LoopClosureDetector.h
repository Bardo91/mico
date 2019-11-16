//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef MICO_BASE_MAP3D_LOOPCLOSUREDETECTOR_H_
#define MICO_BASE_MAP3D_LOOPCLOSUREDETECTOR_H_

#include <mico/base/cjson/json.h>
#include <mico/base/utils/LogManager.h>
#include <opencv2/opencv.hpp>

#include <mico/base/map3d/Dataframe.h>

#include <vector>


namespace mico {
    /// Class that holds the results of the loop closure detection. 
    struct LoopResult{
        bool found = false;
        int matchId = -1;
    };
    
    template <DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    class LoopClosureDetector: public LoggableInterface<DebugLevel_, OutInterface_>{
    public:

        /// Initialize loop closure detector. Each implementation has specific configurations.
        virtual bool init(cjson::Json &_config) = 0;

        /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
        virtual LoopResult appendCluster(cv::Mat &_image, int _id) = 0;

        /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
        virtual LoopResult appendCluster(const std::vector<cv::KeyPoint> &_kps, const cv::Mat &_descriptors, int _id) = 0;

        template<typename PointType_>
        std::vector<std::shared_ptr<Dataframe<PointType_>>> findPath(
                    std::shared_ptr<Dataframe<PointType_>> _init, 
                    std::shared_ptr<Dataframe<PointType_>> _end );
    };

}  // namespace mico 

#include <mico/base/map3d/LoopClosureDetector.inl>

#endif