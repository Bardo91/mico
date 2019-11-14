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

#include <mico/flow/blocks/processors/BlockDarknet.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

#include <chrono>
namespace mico{

    BlockDarknet::BlockDarknet(){
        
        iPolicy_ = new Policy({"color","dataframe"});

        opipes_["color"] = new OutPipe("color");
        opipes_["v_entity"] = new OutPipe("v_entity");

        iPolicy_->registerCallback({"color"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        if(hasParameters_){
                                            cv::Mat image;
                                            // check data received
                                            try{
                                                image = std::any_cast<cv::Mat>(_data["color"]).clone();
                                            }catch(std::exception& e){
                                                std::cout << "Failure Darknet. " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }
                                            // vector of detected entities 
                                            std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities;

                                            // get image detections
                                            auto detections = detector_.detect(image);
                                            // detection -> label, confidence, left, top, right, bottom
                                            for(auto &detection: detections){
                                                // confidence threshold 
                                                if(detection[1]>confidenceThreshold){
                                                    std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>> e(new mico::Entity<pcl::PointXYZRGBNormal>(
                                                         numEntities, detection[0], detection[1], {detection[2],detection[3],detection[4],detection[5]}));                                                                                          
                                                    entities.push_back(e);
                                                    numEntities++;
                                                    cv::Rect rec(detection[2], detection[3], detection[4] -detection[2], detection[5]-detection[3]);
                                                    //cv::putText(image, "Confidence" + std::to_string(detection[1]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    cv::putText(image, "ObjectId: " + std::to_string(detection[0]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    cv::rectangle(image, rec, cv::Scalar(0,255,0));
                                                }
                                            }

                                            // send image with detections
                                            if(opipes_["color"]->registrations() !=0 )
                                                opipes_["color"]->flush(image);
                                            // send entities
                                            if(opipes_["v_entity"]->registrations() !=0 )
                                                opipes_["v_entity"]->flush(entities);
                                            
                                        }else{
                                            std::cout << "No weights and cfg provided to Darknet\n";
                                        }
                                        #endif
                                        idle_ = true;
                                    }
                                });

        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        if(hasParameters_){
                                            cv::Mat image;
                                            std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> df = nullptr;

                                            // check data received
                                            try{
                                                df = std::any_cast<std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>>(_data["dataframe"]);
                                                image = df->leftImage().clone();
                                                
                                            }catch(std::exception& e){
                                                std::cout << "Failure Darknet dataframe registration. " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }
                                            auto strt = std::chrono::steady_clock::now();

                                            // vector of detected entities 
                                            std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities;

                                            // get image detections
                                            auto detections = detector_.detect(image);
                                            // detection -> label, confidence, left, top, right, bottom

                                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr featureCloud = df->featureCloud();
                                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr denseCloud = df->cloud();
                                            std::vector<cv::Point2f> featureProjections = df->featureProjections();

                                            for(auto &detection: detections){
                                               if(detection[1]>confidenceThreshold){
                                                    std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>> e(new mico::Entity<pcl::PointXYZRGBNormal>(
                                                         numEntities, df->id(), detection[0], detection[1], {detection[2],detection[3],detection[4],detection[5]}));  
                                                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr entityCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
                                                    std::vector<cv::Point2f> entityProjections;

                                                    if(featureProjections.size() > 0 && featureCloud != nullptr){
                                                        // for(auto it = featureProjections.begin(); it != featureProjections.end(); it++ ){
                                                        //     if( it->x > detection[2] && it->x < detection[4] && it->y > detection[3] && it->y < detection[5]){
                                                        //         entityProjections.push_back(*it);
                                                                
                                                        //         // auto index = it - featureProjections.begin();
                                                        //         // entityCloud->push_back(featureCloud->points[index]);
                                                                
                                                        //         // mising descriptors
                                                        //     }
                                                        // }

                                                        for (int dy = detection[3]; dy < detection[5]; dy++) {
                                                            for (int dx = detection[2]; dx < detection[4]; dx++) {
                                                                pcl::PointXYZRGBNormal p = denseCloud->at(dx,dy);
                                                                if(p.x != NAN && p.y != NAN && p.z != NAN)
                                                                    entityCloud->push_back(p);
                                                            }
                                                        }
                                                        e->projections(df->id(), entityProjections);
                                                        e->cloud(df->id(), entityCloud);
                                                        e->computePCA(df->id());
                                                        entities.push_back(e);
                                                        numEntities++;
                                                    }
                                                    
                                                    //cv::Rect rec(detection[2], detection[3], detection[4] -detection[2], detection[5]-detection[3]);
                                                    ////cv::putText(image, "Confidence" + std::to_string(detection[1]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    //cv::putText(image, "ObjectId: " + std::to_string(detection[0]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    //cv::rectangle(image, rec, cv::Scalar(0,255,0));

                                                }
                                            }
                                            // send entities
                                            if(opipes_["v_entity"]->registrations() !=0 )
                                                opipes_["v_entity"]->flush(entities);
                                            // send image with detections
                                            if(opipes_["color"]->registrations() !=0 )
                                                opipes_["color"]->flush(image);
                                            //auto end = std::chrono::steady_clock::now();
                                            //printf("Detector: Elapsed time in milliseconds : %i", std::chrono::duration_cast<std::chrono::milliseconds>(end - strt).count());
                                        }else{
                                            std::cout << "No weights and cfg provided to Darknet\n";
                                        }
                                        #endif
                                        idle_ = true;
                                    }
                                });
    }

    bool BlockDarknet::configure(std::unordered_map<std::string, std::string> _params){        
        #ifdef HAS_DARKNET
        std::string cfgFile;
        std::string weightsFile;
        for(auto &p: _params){
            if(p.first == "cfg"){
                cfgFile = p.second;
            }else if(p.first == "weights"){
                weightsFile = p.second;
            }else if(p.first == "confidence threshold"){
                if(p.second.compare("confidence threshold"))
                    confidenceThreshold = stof(p.second);
            }   
        }

        // cfg file provided?
        if(!cfgFile.compare("cfg")){
            std::cout << "Cfg not provided \n";                    
            cfgFile = getenv("HOME") + std::string("/.mico/downloads/yolov3-tiny.cfg");
            // cfg file already downloaded?
            if(!std::experimental::filesystem::exists(cfgFile)){
                std::cout << "Downloading yolov3-tiny.cfg \n";
                system("wget -P ~/.mico/downloads https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg");
            }
        }   

        // weights file provided?
        if(!weightsFile.compare("weights")){    
            std::cout << "Weights not provided \n";                    
            weightsFile = getenv("HOME") + std::string("/.mico/downloads/yolov3-tiny.weights");
            // cfg file already downloaded?
            if(!std::experimental::filesystem::exists(weightsFile)){
                std::cout << "Downloading yolov3-tiny.weights \n";
                system("wget -P ~/.mico/downloads https://pjreddie.com/media/files/yolov3-tiny.weights");
            }
        }



        std::cout << "cfg file : " << cfgFile << "\n";
        std::cout << "weightsFile : " << weightsFile << "\n";
        std::cout << "confidence threshold : " << confidenceThreshold << "\n";

        hasParameters_ = true;  
        if(detector_.init(cfgFile,weightsFile)){
            std::cout << "Good init\n";
            return true;
        }
        else{
            std::cout << "Detector: Bad input arguments\n";
            return false;
        }
        #else
        return false;
        #endif
    }
    
    std::vector<std::string> BlockDarknet::parameters(){
        return {"cfg","weights", "confidence threshold"};
    }


}
