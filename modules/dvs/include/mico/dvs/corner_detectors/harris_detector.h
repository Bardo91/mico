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

#ifndef MICO_DVS_CORNERDETECTORS_HARRIS_DETECTOR_H_
#define MICO_DVS_CORNERDETECTORS_HARRIS_DETECTOR_H_

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include "mico/dvs/corner_detectors/detector.h"

#include "mico/dvs/corner_detectors/local_event_queues.h"
#include "mico/dvs/corner_detectors/distinct_queue.h"

namespace mico{

  class HarrisDetector : public Detector{
  public:
    HarrisDetector();
    virtual ~HarrisDetector();

    bool isFeature(const dvs_msgs::Event &e);

    double getLastScore() const {
      return lastScore_;
    }

  private:
    // methods
    void updateQueue(const int x, const int y, const dvs_msgs::Event &e);
    double getHarrisScore(int x, int y, bool polarity);

    // queues
    LocalEventQueues* queues_;

    // parameters
    int queueSize_;
    int windowSize_;
    int kernelSize_;
    static const int sensorWidth_  = 240;
    static const int sensorHeight_ = 180;
    double harrisThreshold_;

    double lastScore_;

    // kernels
    Eigen::MatrixXd Gx_, h_;
    int factorial(int n) const;
    int pasc(int k, int n) const;
  };


} // namespace

#endif