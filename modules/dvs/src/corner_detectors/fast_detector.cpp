//---------------------------------------------------------------------------------------------------------------------
//  CORNER DETECTOR https://github.com/uzh-rpg/rpg_corner_events
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018
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

#include "mico/dvs/corner_detectors/fast_detector.h"

namespace mico{

  FastDetector::FastDetector() :
    circle3_ {{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    circle4_ {{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}
  {
    detectorName_ = "FAST";

    // allocate SAE matrices
    sae_[0] = Eigen::MatrixXd::Zero(sensorHeight_, sensorWidth_);
    sae_[1] = Eigen::MatrixXd::Zero(sensorHeight_, sensorWidth_);
  }

  FastDetector::~FastDetector(){
  }

  bool FastDetector::isFeature(const dvs_msgs::Event &e){
    // update SAE
    const int pol = e.polarity ? 1 : 0;
    sae_[pol](e.x, e.y) = e.ts.toSec();

    const int max_scale = 1;

    // only check if not too close to border
    const int cs = max_scale*4;
    if (e.x < cs || e.x >= sensorWidth_-cs || e.y < cs || e.y >= sensorHeight_-cs){
      return false;
    }

    bool found_streak = false;

    for (int i=0; i<16; i++){
      for (int streak_size = 3; streak_size<=6; streak_size++){
        // check that streak event is larger than neighbor
        if (sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]) <
                              sae_[pol](e.x+circle3_[(i-1+16)%16][0], e.y+circle3_[(i-1+16)%16][1]))
          continue;

        // check that streak event is larger than neighbor
        if (sae_[pol](e.x+circle3_[(i+streak_size-1)%16][0], e.y+circle3_[(i+streak_size-1)%16][1]) <
                  sae_[pol](e.x+circle3_[(i+streak_size)%16][0], e.y+circle3_[(i+streak_size)%16][1]))
          continue;

        double min_t = sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]);
        for (int j=1; j<streak_size; j++){
          const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<16; j++){
          const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);

          if (tj >= min_t){
            did_break = true;
            break;
          }
        }

        if (!did_break){
          found_streak = true;
          break;
        }

      }
      if (found_streak){
        break;
      }
    }

    if (found_streak){
      found_streak = false;
      for (int i=0; i<20; i++){
        for (int streak_size = 4; streak_size<=8; streak_size++){
          // check that first event is larger than neighbor
          if (sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]) <  
                          sae_[pol](e.x+circle4_[(i-1+20)%20][0], e.y+circle4_[(i-1+20)%20][1]))
            continue;

          // check that streak event is larger than neighbor
          if (sae_[pol](e.x+circle4_[(i+streak_size-1)%20][0], e.y+circle4_[(i+streak_size-1)%20][1]) <          
                          sae_[pol](e.x+circle4_[(i+streak_size)%20][0], e.y+circle4_[(i+streak_size)%20][1]))
            continue;

          double min_t = sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]);
          for (int j=1; j<streak_size; j++){
            const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
            if (tj < min_t)
              min_t = tj;
          }

          bool did_break = false;
          for (int j=streak_size; j<20; j++){
            const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
            if (tj >= min_t){
              did_break = true;
              break;
            }
          }

          if (!did_break){
            found_streak = true;
            break;
          }
        }
        if (found_streak){
          break;
        }
      }
    }

    return found_streak;
  }

} // namespace
