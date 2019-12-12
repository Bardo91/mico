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

#include <mico/dvs/corner_detectors/detector.h>

namespace mico{

  Detector::Detector(bool detect){
    totalCorners_ = 0;
    totalEvents_  = 0;
    totalTime_    = 0;

    // interface
    if (detect)
    {
      // feature_pub_ = nh_.advertise<dvs_msgs::EventArray>("/feature_events", 1);
      // event_sub_ = nh_.subscribe("/dvs/events", 0, &Detector::eventCallback, this);
    }
  }

  Detector::~Detector(){
  }

  void Detector::eventCallback(const dvs_msgs::EventArray &_msg){

    dvs_msgs::EventArray feature_msg;
    feature_msg.header = _msg.header;
    feature_msg.width = _msg.width;
    feature_msg.height = _msg.height;

    utils::time::Timer<std::chrono::nanoseconds> timer;
    for (const auto e : _msg.events){
      if (isFeature(e)){ // call to FAST or Harris
        feature_msg.events.push_back(e);
      }
    }
    const auto elapsed_time_nsecs = timer.toc();

    // global stats
    totalTime_    += elapsed_time_nsecs;
    totalEvents_  += _msg.events.size();
    totalCorners_ += feature_msg.events.size();

    // publish feature events
    cornersDetected_ = feature_msg;

    // stats
    const int numEvents_ = _msg.events.size();
    if (numEvents_ > 0){
      const int numFeatures_ = feature_msg.events.size();
      const float reductionRate_ = 100.*(1.-numFeatures_/(float) numEvents_);
      const float reductionFactor_ = numEvents_/(float) numFeatures_;
      const float eventsPerSecond_ = float(numEvents_)/(elapsed_time_nsecs/1e9);
      const float nsPerEvent_ = elapsed_time_nsecs/float(numEvents_);
      ROS_INFO("%s reduction rate: %.3f%% (%.0fx). Speed: %.0f e/s / %.0f ns/e.",
        detectorName_.c_str(), reductionRate_, reductionFactor_,
        eventsPerSecond_, nsPerEvent_);
    }
    else
    {
      ROS_INFO("%s reduction rate: No events.", detectorName_.c_str());
    }
  }

} // namespace
