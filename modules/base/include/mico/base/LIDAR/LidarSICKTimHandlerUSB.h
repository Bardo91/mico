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


#ifndef MICO_BASE_LIDAR_LIDARSICKTIMHANDLERUSB_H_
#define MICO_BASE_LIDAR_LIDARSICKTIMHANDLERUSB_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include "LidarSICKTim517_parser.h"

namespace mico{

    class LidarSICKTimHandlerUSB{

    public:
        LidarSICKTimHandlerUSB(LidarParser* _parser, int _deviceNumber);

        /// Send a SOPAS command to the scanner that should cause a soft reset
        /**
         * \returns true if reboot command was accepted, false otherwise
         */
        bool rebootScanner();
        
        int init();
        int loopOnce();

    private:
        int init_device();
        int init_scanner();
        int stop_scanner();
        int close_device();

        
        /// Send a SOPAS command to the device and print out the response to the console.
        /**
         * \param [in] request the command to send.
         * \param [out] reply if not NULL, will be filled with the reply package to the command.
         */
        int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply);

        /// Read a datagram from the device.
        /**
         * \param [in] receiveBuffer data buffer to fill
         * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
         * \param [out] actual_length the actual amount of data written
         */
        int get_datagram(unsigned char* _receiveBuffer, int _bufferSize, int* _actualLength);
        
        /// Converts reply from sendSOPASCommand to string
        /**
         * \param [in] reply reply from sendSOPASCommand
         * \returns reply as string with special characters stripped out
         */
        static std::string replyToString(const std::vector<unsigned char> &reply);

        bool isCompatibleDevice(const std::string identStr) const;

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

    };
}

#endif