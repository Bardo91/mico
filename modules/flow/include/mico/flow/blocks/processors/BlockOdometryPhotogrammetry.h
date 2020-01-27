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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKODOMETRYPHOTOGRAMMETRY_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKODOMETRYPHOTOGRAMMETRY_H_

#include <flow/Block.h>
#include <mico/base/map3d/OdometryPhotogrammetry.h>

namespace mico{

    class BlockOdometryPhotogrammetry: public flow::Block{
    public:
        virtual std::string name() override {return "Odometry Photogrammetry";}

        BlockOdometryPhotogrammetry();
        // ~BlockOdometryPhotogrammetry(){};

        bool configure(std::unordered_map<std::string, std::string> _params) override;
        std::vector<std::string> parameters() override;

        std::string description() const override {return    "Block for visual odometry estimation using zenital images.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};
    private:
        void callbackOdometry(flow::DataFlow _data);

        bool computePointCloud(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> &_df);
        bool pinHoleModel(float cam_height , std::vector<cv::KeyPoint> keypoints, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &OutputPointCloud);
    private:

        bool hasCalibration = false;

        bool hasPrev_ = false;
        int nextDfId_ = 0;
        cv::Ptr<cv::ORB> featureDetector_ ;
        std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> currentKeyframe_ = nullptr;
        std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> prevDf_ = nullptr;
        
        OdometryPhotogrammetry<pcl::PointXYZRGBNormal, mico::DebugLevels::Debug , OutInterfaces::Cout> odom_;
        bool idle_ = true;
        
        cv::Mat matrixLeft_, distCoefLeft_;
        bool savedFirstAltitude_ = false;
        float initSLAMAltitude_ = 5.0;
        float firstAltitude_;
        float altitude_;

        std::map<int,std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>> memoryDf_; 
    };

}

#endif