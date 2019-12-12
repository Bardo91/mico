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

#ifndef MICO_DVS_CORNERDETECTORS_DETECTOR_H_
#define MICO_DVS_CORNERDETECTORS_DETECTOR_H_

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <mico/dvs/corner_detectors/timer.h>

namespace mico{

  class Detector{
    public:
      Detector(bool connect = true);
      virtual ~Detector();

      // check if event
      virtual bool isFeature(const dvs_msgs::Event &e) = 0;

      virtual dvs_msgs::EventArray cornersDetected(){
        return cornersDetected_;
      }

      // interface
      void eventCallback(const dvs_msgs::EventArray &_msg);

    protected:
      std::string detectorName_;

    private:

      // statistics
      double totalTime_;
      int totalEvents_, totalCorners_ , numEvents_ , numFeatures_;
      float reductionRate_, reductionFactor_ , eventsPerSecond_ , nsPerEvent_;

      dvs_msgs::EventArray cornersDetected_;
  };


} // namespace

#endif
