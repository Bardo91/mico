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

  Detector::Detector(){
    totalCorners_ = 0;
    totalEvents_  = 0;
    totalTime_    = 0;
  }

  Detector::~Detector(){
  }

  void Detector::eventCallback(const dvs_msgs::EventArray &_msg){

    dvs_msgs::EventArray feature_msg;
    feature_msg.header = _msg.header;
    feature_msg.width = _msg.width;
    feature_msg.height = _msg.height;

    for (const auto e : _msg.events){
  
      if (isFeature(e)){ // call to FAST or Harris
        feature_msg.events.push_back(e);
      }
    }
    // publish feature events
    cornersDetected_ = feature_msg;

    // global stats
    // totalTime_    += elapsed_time_nsecs;
    // totalEvents_  += _msg.events.size();
    // totalCorners_ += feature_msg.events.size();

    // events stats
    // numEvents_ = _msg.events.size();
    // numFeatures_ = feature_msg.events.size();
    // reductionRate_ = 100.*(1.-numFeatures_/(float) numEvents_);
    // reductionFactor_ = numEvents_/(float) numFeatures_;
    // eventsPerSecond_ = float(numEvents_)/(elapsed_time_nsecs/1e9);
    // nsPerEvent_ = elapsed_time_nsecs/float(numEvents_); 
    
  }

} // namespace
