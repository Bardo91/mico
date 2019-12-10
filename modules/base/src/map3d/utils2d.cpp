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


#include <opencv2/opencv.hpp>
#include <vector>
#include <mico/base/map3d/utils2d.h>
#include <algorithm>

namespace mico {

    bool matchDescriptorsBF(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers,double _mk_nearest_neighbors,double _mFactorDescriptorDistance){
        if(_des1.rows == 0 || _des2.rows == 0){
            return false;
        }
        
        std::vector<std::vector<cv::DMatch>> matches12, matches21;
        cv::BFMatcher featureMatcher(cv::NORM_HAMMING,true);
        featureMatcher.knnMatch(_des1, _des2, matches12,_mk_nearest_neighbors);
        featureMatcher.knnMatch(_des2, _des1, matches21,_mk_nearest_neighbors);
        // double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints   --- 666 LOOK FOR RATIO TEST AND IMPROVE THIS SHIT!
        // for( int i = 0; i < _des1.rows; i++ ) {
        //     for(int j = 0; j < matches12[i].size(); j++){
        //         double dist = matches12[i][j].distance;
        //         if( dist < min_dist ) min_dist = dist;
        //         if( dist > max_dist ) max_dist = dist;
        //     }
        // }

        // min_dist = min_dist==0 ? 2 : min_dist;
        std::vector<cv::DMatch> matches12fil, matches21fil; // 666 POSSIBLE OPTIMIZATION
        // RADIO TEST
        if(_mk_nearest_neighbors == 1){
            matches12fil = matches12[0];
            matches21fil = matches21[0];    
        }else if(_mk_nearest_neighbors == 2){
            for(auto &matches: matches12){
                if(matches[0].distance < matches[1].distance * _mFactorDescriptorDistance){
                    matches12fil.push_back(matches[0]);
                }
            }

            for(auto &matches: matches21){
                if(matches[0].distance < matches[1].distance * _mFactorDescriptorDistance){
                    matches21fil.push_back(matches[0]);
                }
            }
        }else{
            std::cout << "knn neighbors need to be 1 or 2" <<std::endl;
            return false;
        }

        // // symmetry test.
        // for(std::vector<cv::DMatch>::iterator it12 = matches12fil.begin(); it12 != matches12fil.end(); it12++){
        //     for(std::vector<cv::DMatch>::iterator it21 = matches21fil.begin(); it21 != matches21fil.end(); it21++){
        //         if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
        //             _inliers.push_back(*it12);
        //             break;
        //         }
        //     }
        // }

        // symmetry test.
        for( int i = 0; i < _des1.rows; i++ )
            for(std::vector<cv::DMatch>::iterator it12 = matches12[i].begin(); it12 != matches12[i].end(); it12++){
                for( int j = 0; j < _des2.rows; j++ )
                    for(std::vector<cv::DMatch>::iterator it21 = matches21[j].begin(); it21 != matches21[j].end(); it21++){
                        if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                            float factor = it12->distance/it21->distance;
                            factor = factor < 1? 1/factor : factor;
                            if(factor < _mFactorDescriptorDistance){
                                _inliers.push_back(*it12);
                                break;
                            }
                        }
                    }
            }
    return true;
	}

    bool matchDescriptorsKDT(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers,double _mk_nearest_neighbors){    
        cv::Ptr<cv::DescriptorMatcher> matcherKDT = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(6,12, 2) ); // 12,20,2
		
		matcherKDT->add(_des2);
  		matcherKDT->train();
		std::vector<std::vector<cv::DMatch>> matches12, matches21;
		matcherKDT->knnMatch(_des1, _des2, matches12, 2 ); // _mk_nearest_neighbors
        matcherKDT->knnMatch(_des2, _des1, matches21, 2 ); // _mk_nearest_neighbors

		// Filter matches using the Lowe's ratio test
		const float ratio_thresh = 0.83f;
		std::vector<cv::DMatch> good_12,good_21;
		for (size_t i = 0; i < matches12.size(); i++){
			if (matches12[i][0].distance < ratio_thresh * matches12[i][1].distance){
				good_12.push_back(matches12[i][0]);
			}
		}
		for (size_t i = 0; i < matches21.size(); i++){
			if (matches21[i][0].distance < ratio_thresh * matches21[i][1].distance){
				good_21.push_back(matches21[i][0]);
			}
		}

        // symmetry test.
        for(std::vector<cv::DMatch>::iterator it12 = good_12.begin(); it12 != good_12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = good_21.begin(); it21 != good_21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    _inliers.push_back(*it12);
                    break;
                }
            }
        }
		if ((int)_inliers.size() > 0){
			// printf("symetric matches using LSH: %i \n",_inliers.size());
			return true;
		}else{
			return false;
		}
	}
}

