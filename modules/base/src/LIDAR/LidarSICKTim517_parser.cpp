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
//
// Adaptation of code taken from: https://github.com/uos/sick_tim
//

#include <mico/base/LIDAR/LidarSICKTim517_parser.h>

namespace mico{

    LidarSICKTim571Parser::LidarSICKTim571Parser(){
        overrideRangeMin_ = 0.05;
        overrideRangeMax_ = 10.0;
        overrideTimeIncrement_ = -1.0;
    }

    int LidarSICKTim571Parser::parse_datagram(char* _datagram, size_t _datagramLength,sensor_msgs::LaserScan &_msg){
        // general message structure:
        //
        // - message header   20 fields
        // - DIST1 header      6 fields
        // - DIST1 data        N fields
        // - RSSI included?    1 field
        // - RSSI1 header      6 fields (optional)
        // - RSSI1 data        N fields (optional)
        // - footer         >= 5 fields, depending on number of spaces in device label
        static const size_t HEADER_FIELDS = 26;
        static const size_t MIN_FOOTER_FIELDS = 5;
        char* cur_field;
        size_t count;

        // Reserve sufficient space
        std::vector<char *> fields;
        fields.reserve(_datagramLength / 2);

        // ----- only for debug output
        char datagram_copy[_datagramLength + 1];
        strncpy(datagram_copy, _datagram, _datagramLength); // datagram will be changed by strtok
        datagram_copy[_datagramLength] = 0;

        // ----- tokenize
        count = 0;
        cur_field = strtok(_datagram, " ");

        while (cur_field != NULL){
            fields.push_back(cur_field);
            cur_field = strtok(NULL, " ");
        }

        count = fields.size();

        // Validate header. Total number of tokens is highly unreliable as this may
        // change when you change the scanning range or the device name using SOPAS ET
        // tool. The header remains stable, however.
        if (count < HEADER_FIELDS + 1 + MIN_FOOTER_FIELDS){
            printf("received less fields than minimum fields (actual: %zu, minimum: %zu), ignoring scan", count, HEADER_FIELDS + 1 + MIN_FOOTER_FIELDS);
            printf("are you using the correct node? (124 --> sick_tim310_1130000m01, > 32 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
            // printf("received message was: %s", datagram_copy);
            return ExitError;
        }
        if (strcmp(fields[15], "0")){
            printf("Field 15 of received data is not equal to 0 (%s). Unexpected data, ignoring scan", fields[15]);
            return ExitError;
        }
        if (strcmp(fields[20], "DIST1")){
            printf("Field 20 of received data is not equal to DIST1i (%s). Unexpected data, ignoring scan", fields[20]);
            return ExitError;
        }

        // More in depth checks: check data length and RSSI availability
        // 25: Number of data (<= 10F)
        unsigned short int number_of_data = 0;
        sscanf(fields[25], "%hx", &number_of_data);

        if (number_of_data < 1 || number_of_data > 811){
            printf("Data length is outside acceptable range 1-811 (%d). Ignoring scan", number_of_data);
            return ExitError;
        }
        if (count < HEADER_FIELDS + number_of_data + 1 + MIN_FOOTER_FIELDS){
            printf("Less fields than expected (expected: >= %zu, actual: %zu). Ignoring scan",
                    HEADER_FIELDS + number_of_data + 1 + MIN_FOOTER_FIELDS, count);
            return ExitError;
        }
        printf("Number of data: %d", number_of_data);

        // Calculate offset of field that contains indicator of whether or not RSSI data is included
        size_t rssi_idx = HEADER_FIELDS + number_of_data;
        int tmp;
        sscanf(fields[rssi_idx], "%d", &tmp);
        bool rssi = tmp > 0;
        unsigned short int numberOfRSSIData = 0;
        if (rssi){
            sscanf(fields[rssi_idx + 6], "%hx", &numberOfRSSIData);

            // Number of RSSI data should be equal to number of data
            if (numberOfRSSIData != number_of_data){
                printf("Number of RSSI data (%d) is not equal to number of range data (%d)", numberOfRSSIData, number_of_data);
                return ExitError;
            }

            // Check if the total length is still appropriate.
            // RSSI data size = number of RSSI readings + 6 fields describing the data
            if (count < HEADER_FIELDS + number_of_data + 1 + 6 + numberOfRSSIData + MIN_FOOTER_FIELDS){
                printf("Less fields than expected with RSSI data (expected: >= %zu, actual: %zu). Ignoring scan",
                        HEADER_FIELDS + number_of_data + 1 + 6 + numberOfRSSIData + MIN_FOOTER_FIELDS, count);
                return ExitError;
            }

            if (strcmp(fields[rssi_idx + 1], "RSSI1")){
                printf("Field %zu of received data is not equal to RSSI1 (%s). Unexpected data, ignoring scan", rssi_idx + 1, fields[rssi_idx + 1]);
            }
        }

        // ----- read fields into msg
        _msg.header.frame_id = "map";
        
        ros::Time start_time = ros::Time::now(); // will be adjusted in the end

        // <STX> (\x02)
        // 0: Type of command (SN)
        // 1: Command (LMDscandata)
        // 2: Firmware version number (1)
        // 3: Device number (1)
        // 4: Serial number (eg. B96518)
        // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
        // 7: Telegram counter (eg. 99)
        // 8: Scan counter (eg. 9A)
        // 9: Time since startup (eg. 13C8E59)
        // 10: Time of transmission (eg. 13C9CBE)
        // 11 + 12: Input status (0 0)
        // 13 + 14: Output status (8 0)
        // 15: Reserved Byte A (0)

        // 16: Scanning Frequency (5DC)
        unsigned short scanning_freq = -1;
        sscanf(fields[16], "%hx", &scanning_freq);
        _msg.scan_time = 1.0 / (scanning_freq / 100.0);
        // printf("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, _msg.scan_time);

        // 17: Measurement Frequency (36)
        unsigned short measurement_freq = -1;
        sscanf(fields[17], "%hx", &measurement_freq);
        _msg.time_increment = 1.0 / (measurement_freq * 100.0);
        if (overrideTimeIncrement_ > 0.0){
            // Some lasers may report incorrect measurement frequency
            _msg.time_increment = overrideTimeIncrement_;
        }
        // printf("measurement_freq: %d, time_increment: %f", measurement_freq, _msg.time_increment);

        // 18: Number of encoders (0)
        // 19: Number of 16 bit channels (1)
        // 20: Measured data contents (DIST1)

        // 21: Scaling factor (3F800000)
        // ignored for now (is always 1.0):
        //      unsigned int scaling_factor_int = -1;
        //      sscanf(fields[21], "%x", &scaling_factor_int);
        //
        //      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
        //      // printf("hex: %s, scaling_factor_int: %d, scaling_factor: %f", fields[21], scaling_factor_int, scaling_factor);

        // 22: Scaling offset (00000000) -- always 0
        // 23: Starting angle (FFF92230)
        int starting_angle = -1;
        sscanf(fields[23], "%x", &starting_angle);
        _msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
        // printf("starting_angle: %d, angle_min: %f", starting_angle, _msg.angle_min);

        // 24: Angular step width (2710)
        unsigned short angular_step_width = -1;
        sscanf(fields[24], "%hx", &angular_step_width);
        _msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
        _msg.angle_max = _msg.angle_min + (number_of_data - 1) * _msg.angle_increment;

        // 25: Number of data (<= 10F)
        // This is already determined above in number_of_data

        // adjust angle_min to min_ang config param
        int index_min = 0;
        while (_msg.angle_min + _msg.angle_increment < minAngle_){
            _msg.angle_min += _msg.angle_increment;
            index_min++;
        }

        // adjust angle_max to max_ang config param
        int index_max = number_of_data - 1;
        while (_msg.angle_max - _msg.angle_increment > maxAngle_){
            _msg.angle_max -= _msg.angle_increment;
            index_max--;
        }

        printf("index_min: %d, index_max: %d", index_min, index_max);
        // printf("angular_step_width: %d, angle_increment: %f, angle_max: %f", angular_step_width, _msg.angle_increment, _msg.angle_max);

        // 26..26 + n - 1: Data_1 .. Data_n
        _msg.ranges.resize(index_max - index_min + 1);
        for (int j = index_min; j <= index_max; ++j){
            unsigned short range;
            sscanf(fields[j + HEADER_FIELDS], "%hx", &range);
            if (range == 0)
            _msg.ranges[j - index_min] = std::numeric_limits<float>::infinity();
            else
            _msg.ranges[j - index_min] = range / 1000.0;
        }

        if (intensity_) {
            if (rssi)
            {
                // 26 + n: RSSI data included

                //   26 + n + 1 = RSSI Measured Data Contents (RSSI1)
                //   26 + n + 2 = RSSI scaling factor (3F80000)
                //   26 + n + 3 = RSSI Scaling offset (0000000)
                //   26 + n + 4 = RSSI starting angle (equal to Range starting angle)
                //   26 + n + 5 = RSSI angular step width (equal to Range angular step width)
                //   26 + n + 6 = RSSI number of data (equal to Range number of data)
                //   26 + n + 7 .. 26 + n + 7 + n - 1: RSSI_Data_1 .. RSSI_Data_n
                //   26 + n + 7 + n = unknown (seems to be always 0)
                //   26 + n + 7 + n + 1 = device label included? (0 = no, 1 = yes)
                //   26 + n + 7 + n + 2 .. count - 4 = device label as a length-prefixed string, e.g. 0xA "Scipio_LRF" or 0xB "not defined"
                //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
                //   <ETX> (\x03)
                _msg.intensities.resize(index_max - index_min + 1);
                size_t offset = HEADER_FIELDS + number_of_data + 7;
                for (int j = index_min; j <= index_max; ++j){
                    unsigned short intensity;
                    sscanf(fields[j + offset], "%hx", &intensity);
                    _msg.intensities[j - index_min] = intensity;
                }
            } else {
            printf("Intensity parameter is enabled, but the scanner is not configured to send RSSI values! "
            "Please read the section 'Enabling intensity (RSSI) output' here: http://wiki.ros.org/sick_tim.");
            }
        }

        // 26 + n: RSSI data included
        // IF RSSI not included:
        //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
        //   26 + n + 4 .. count - 4 = device label
        //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
        //   <ETX> (\x03)

        _msg.range_min = overrideRangeMin_;
        _msg.range_max = overrideRangeMax_;

        // ----- adjust start time
        // - last scan point = now  ==>  first scan point = now - number_of_data * time increment
        double start_time_adjusted = start_time.toSec()
                    - number_of_data * _msg.time_increment   // shift backward to time of first scan point
                    + index_min * _msg.time_increment        // shift forward to time of first published scan point
                    + timeOffset_;                   // add time offset (usually negative) to account for USB latency etc.
        if (start_time_adjusted >= 0.0){   // ensure that ros::Time is not negative (otherwise runtime error)
            _msg.header.stamp.fromSec(start_time_adjusted);
        } else {
            printf("ROS time is 0! Did you set the parameter use_sim_time to true?");
        }

        // ----- consistency check
        float expected_time_increment = _msg.scan_time * _msg.angle_increment / (2.0 * M_PI);
        if (fabs(expected_time_increment - _msg.time_increment) > 0.00001){
        printf("The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
            "Expected time_increment: %.9f, reported time_increment: %.9f. "
            "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
            expected_time_increment, _msg.time_increment);
        }

        return ExitSuccess;
    }


}
