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

#ifndef MICO_BASE_LIDAR_LIDARSICKTIMHANDLERUSB_H_
#define MICO_BASE_LIDAR_LIDARSICKTIMHANDLERUSB_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include <mico/base/utils/LogManager.h>

#include "LidarSICKTim517_parser.h"

#include <ros/ros.h>

namespace mico{
    class LidarSICKTimHandlerUSB  : public LoggableInterface<DebugLevels::Debug, OutInterfaces::Cout>{

    public:
        LidarSICKTimHandlerUSB(LidarParser* _parser, int _deviceNumber);
        ~LidarSICKTimHandlerUSB();
        
        int init();
        int loopOnce();

    private:
        int init_device();
        int init_scanner();
        int stop_scanner();
        int close_device();

        
        /// Send a SOPAS command to the device and print out the response to the console.
        // 
        // \param [in] request the command to send.
        // \param [out] reply if not NULL, will be filled with the reply package to the command.
        int sendSOPASCommand(const char* _request, std::vector<unsigned char> * _reply);

        /// Read a datagram from the device.
        /// 
        /// \param [in] receiveBuffer data buffer to fill
        /// \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
        /// \param [out] actual_length the actual amount of data written
        int get_datagram(unsigned char* _receiveBuffer, int _bufferSize, int* _actualLength);
        
        /// Converts reply from sendSOPASCommand to string
        ///
        ///\param [in] reply reply from sendSOPASCommand
        ///\returns reply as string with special characters stripped out
        static std::string replyToString(const std::vector<unsigned char> &_reply);

        bool isCompatibleDevice(const std::string _identStr);

        ssize_t getSOPASDeviceList(libusb_context *_ctx, uint16_t _vendorID, uint16_t _productID, libusb_device ***_list);
        void freeSOPASDeviceList(libusb_device **_list);

    private:
        static const unsigned int USB_TIMEOUT = 1000; // milliseconds

        // libusb
        libusb_context *ctx_;
        libusb_device **devices_;
        libusb_device_handle *deviceHandle_;
        ssize_t numberOfDevices_;
        int deviceNumber_;

        // Parser
        LidarParser* parser_ = nullptr;

        // ROS
        ros::NodeHandle nh_;
        ros::Publisher lidarPub_;

    };
}

#endif