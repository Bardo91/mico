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

#include <mico/flow/blocks/processors/BlockOdometryPhotogrammetry.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>

namespace mico{

    BlockOdometryPhotogrammetry::BlockOdometryPhotogrammetry(){
        createPolicy({     {"Color Image", "image"},
                            {"Altitude", "float"},
                            {"Keyframe", "dataframe"}});

        createPipe("Dataframe Positioned", "dataframe");

        featureDetector_ = cv::ORB::create(2000);
        
        registerCallback({"Color Image"}, 
                                 [this](flow::DataFlow _data){
                                    this->callbackOdometry(_data);
                                });

        registerCallback({"Altitude"},
            [&](flow::DataFlow _data){
                    altitude_ = _data.get<float>("Altitude");
                    if (!savedFirstAltitude_){
                        firstAltitude_ = altitude_;
                        savedFirstAltitude_ = true;
                    }
                });
        
        
        registerCallback({"Keyframe"}, 
                                [&](flow::DataFlow _data){
                                        currentKeyframe_ = _data.get<std::shared_ptr< mico::Dataframe<pcl::PointXYZRGBNormal>>>("Keyframe");
                                    }
                                );

    }


    bool BlockOdometryPhotogrammetry::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first == "calibration"){
                if(param.second == "")
                    return false;
                    
                cv::FileStorage fs(param.second, cv::FileStorage::READ);
                fs["MatrixLeft"]            >> matrixLeft_;
                fs["DistCoeffsLeft"]        >> distCoefLeft_;
                
                hasCalibration = true;
                return true;
            }else if(param.first == "params_json"){
                std::ifstream file(param.second);
                if (!file.is_open()) {
                    std::cout << "Cannot open file." << std::endl;
                    return false;
                }
                cjson::Json configFile;
                if (!configFile.parse(file)) {
                    std::cout << "Cannot parse config file." << std::endl;
                    return false;
                }
                if (!odom_.init(configFile["registrator_params"])) {
                    std::cout << "Error initializing odometry parameters" << std::endl;
                    return false;
                }
            }
        }

        return false;

    }
    
    std::vector<std::string> BlockOdometryPhotogrammetry::parameters(){
        return {"calibration", "params_json"};
    }

    void BlockOdometryPhotogrammetry::callbackOdometry(flow::DataFlow _data){
        if(idle_){
            if(hasCalibration){
                std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> df(new Dataframe<pcl::PointXYZRGBNormal>(nextDfId_));
                nextDfId_++;
                try{
                    df->leftImage(_data.get<cv::Mat>("Color Image"));
                    df->intrinsics(matrixLeft_);
                    df->distCoeff(distCoefLeft_);
                }catch(std::exception& e){
                    std::cout << "Failure Odometry Photogrammetry " <<  e.what() << std::endl;
                    idle_ = true;
                    return;
                }
                if(!computePointCloud(df))
                    return;

                if(df->featureDescriptors().rows == 0)
                    return;

                Dataframe<pcl::PointXYZRGBNormal>::Ptr referenceFrame;
                if(currentKeyframe_ != nullptr) // If there is a keyframe, kf based odometry
                    referenceFrame = currentKeyframe_;
                else  // Just sequential odometry
                    referenceFrame = prevDf_;
                
                if(odom_.computeOdometry(referenceFrame, df)){
                    // memoryDf_[df->id()] = df;   // 666 safety reasons, but memory consumption.
                    getPipe("Dataframe Positioned")->flush(df);  
                }
                prevDf_ = df;

            }else{
                std::cout << "Please, configure Odometry Photogrammetry with the path to the calibration file {\"Calibration\":\"/path/to/file\"}" << std::endl;
            }
            idle_ = true;
        }
    }


    bool BlockOdometryPhotogrammetry::computePointCloud(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> &_df){
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> kpts;
        cv::Mat leftGrayUndistort;

        cv::cvtColor(_df->leftImage(), leftGrayUndistort, cv::ColorConversionCodes::COLOR_BGR2GRAY);
        featureDetector_->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);
        _df->featureDescriptors(descriptors.clone());
        
        if (kpts.size() < 8) {
            return false;
        }
        
        // bad SLAM inicialization
        if (altitude_ < initSLAMAltitude_ ){
            printf("Actual altitude  %f m, SLAM inicializate when %f m \n",altitude_, initSLAMAltitude_);
            return false;
        }
        // Create feature cloud
        _df->featureCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>()));
        _df->cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>()));
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        if(pinHoleModel(altitude_,kpts, cloud)){

            std::vector<cv::Point2f> projs;
            projs.resize(kpts.size());
            for (unsigned k = 0; k < kpts.size(); k++) {
                projs[k] = kpts[k].pt;
            }
            _df->featureCloud(cloud);
            _df->featureProjections(projs);
            
            pcl::copyPointCloud(*(_df->featureCloud()) , *(_df->cloud()));
        }else{
            return false;
        }
        
        return true;
    }


    bool BlockOdometryPhotogrammetry::pinHoleModel(float _altitude ,std::vector<cv::KeyPoint> keypoints, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &OutputPointCloud){
        const float cx = matrixLeft_.at<float>(0,2);
        const float cy = matrixLeft_.at<float>(1,2);    // 666 Move to member? faster method....
        const float fx = matrixLeft_.at<float>(0,0);
        const float fy = matrixLeft_.at<float>(1,1);
        
        for(unsigned ii = 0; ii < keypoints.size(); ii++){
            pcl::PointXYZRGBNormal p;
            p.z = - (_altitude) ;
            p.x = -( ( keypoints[ii].pt.x - cx )/fx ) * (-p.z);
            p.y =  ( ( keypoints[ii].pt.y - cy )/fy ) * (-p.z);
            p.r = 255; p.g = 255; p.b = 255;
    
            OutputPointCloud->points.push_back(p);
        }
        if (OutputPointCloud->points.size() == 0){
            return false;
        }
        return true;
    }
}

