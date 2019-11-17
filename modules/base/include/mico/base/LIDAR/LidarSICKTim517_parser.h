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


#ifndef MICO_BASE_LIDAR_LIDARSICKTIM571_PARSER_H_
#define MICO_BASE_LIDAR_LIDARSICKTIM571_PARSER_H_

#include <mico/base/LidarParser.h>

namespace mico{
    class LidarSICKTim571Parser : public LidarParser{
    
        public:
            LidarSICKTim571Parser();
            virtual ~LidarSICKTim571Parser(){};

            virtual int parse_datagram(char* _datagram, size_t _datagramLength,sensor_msgs::LaserScan &_msg);

            void set_range_min(float _min);
            void set_range_max(float _max);
            void set_time_increment(float _time);

        private:
            float override_range_min_, override_range_max_;
            float override_time_increment_;
            float maxAngle_ = 0.75 * M_PI;
            float minAngle_ =-0.75 * M_PI;
            bool intensity_ = true;
            double timeOffset_ = -0.001;

    };
}

#endif