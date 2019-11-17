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


#include <mico/base/LIDAR/LidarSICKTimHandlerUSB.h>

namespace mico{

    LidarSICKTimHandlerUSB::LidarSICKTimHandlerUSB(LidarParser* _parser, int _deviceNumber){
        parser_ = _parser;
        deviceNumber_ = _deviceNumber;
        ctx_             = nullptr;
        numberOfDevices_ = NULL;
        devices_         = nullptr;
        deviceHandle_   = nullptr;


        // init publisher handler.. eg. can be ROS publisher
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::string LidarSICKTimHandlerUSB::replyToString(const std::vector<unsigned char> &reply){
        std::string reply_str;
        for (std::vector<unsigned char>::const_iterator it = reply.begin(); it != reply.end(); it++){
            if (*it > 13){ // filter control characters for display
                reply_str.push_back(*it);
            }
        }
        return reply_str;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool LidarSICKTimHandlerUSB::isCompatibleDevice(const std::string identStr) const{
        char device_string[7];
        int version_major = -1;
        int version_minor = -1;

        if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
                    &version_major, &version_minor) == 3
                    && strncmp("TiM3", device_string, 4) == 0
                    && version_major >= 2 && version_minor >= 50){
            std::cout << "This scanner model/firmware combination does not support ranging output! \n";
            printf("This is a %s, firmware version %d.%d", device_string, version_major, version_minor);

            return false;
        }

        return true;
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    /**
     * Send a SOPAS command to the device and print out the response to the console.
     */
    int LidarSICKTimHandlerUSB::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply){
      if (deviceHandle_ == NULL) {
        printf("LIBUSB - device not open");
        return ExitError;
      }

      int result = 0;
      unsigned char receiveBuffer[65536];

      /*
       * Write a SOPAS variable read request to the device.
       */
      ROS_DEBUG("LIBUSB - Write data... %s", request);

      int actual_length = 0;
      int requestLength = strlen(request);
      result = libusb_bulk_transfer(deviceHandle_, (2 | LIBUSB_ENDPOINT_OUT), (unsigned char*)request, requestLength,
                                    &actual_length, 0);
      if (result != 0 || actual_length != requestLength)
      {
        printf("LIBUSB - Write Error: %i.", result);
        return result;
      }

      /*
       * Read the SOPAS device response with the given timeout.
       */
      result = libusb_bulk_transfer(deviceHandle_, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual_length, USB_TIMEOUT);
      if (result != 0)
      {
        printf("LIBUSB - Read Error: %i.", result);
        return result;
      }

      receiveBuffer[actual_length] = 0;
      printf("LIBUSB - Read data...  %s", receiveBuffer);
      if(reply) {
          reply->clear();
          for(int i = 0; i < actual_length; i++) {
              reply->push_back(receiveBuffer[i]);
          }
      }

      return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::init(){
        int result = init_device();
        if(result != 0) {
            std::cout << "Failed to init device: " << result << std::endl;
            return result;
        }

        result = init_scanner();
        if(result != 0){
            std::cout << "Failed to init scanner: " << result << std::endl;
        }

        return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::init_scanner(){
        /*
        * Read the SOPAS variable 'DeviceIdent' by index.
        */
        const char requestDeviceIdent[] = "\x02sRI0\x03\0";
        std::vector<unsigned char> identReply;
        int result = sendSOPASCommand(requestDeviceIdent, &identReply);
        if (result != 0){
            std::cout << "SOPAS - Error reading variable 'DeviceIdent' \n";
        }

        /*
        * Read the SOPAS variable 'SerialNumber' by name.
        */
        const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
        std::vector<unsigned char> serialReply;
        result = sendSOPASCommand(requestSerialNumber, &serialReply);
        if (result != 0){
            std::cout << "SOPAS - Error reading variable 'SerialNumber' \n";
        }

        // set hardware ID based on DeviceIdent and SerialNumber
        std::string identStr = replyToString(identReply);
        std::string serialStr = replyToString(serialReply);

        if (!isCompatibleDevice(identStr))
            return ExitFatal;

        /*
        * Read the SOPAS variable 'FirmwareVersion' by name.
        */
        const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
        result = sendSOPASCommand(requestFirmwareVersion, NULL);
        if (result != 0){
            std::cout << "SOPAS - Error reading variable 'FirmwareVersion' \n";
        }

        /*
        * Read Device State
        */
        const char requestDeviceState[] = {"\x02sRN SCdevicestate\x03\0"};
        std::vector<unsigned char> deviceStateReply;
        result = sendSOPASCommand(requestDeviceState, &deviceStateReply);
        if (result != 0){
            std::cout << "SOPAS - Error reading variable 'deviceState' \n";
        }
        std::string deviceStateReplyStr = replyToString(deviceStateReply);

        /*
        * Process device state, 0=Busy, 1=Ready, 2=Error
        * If configuration parameter is set, try resetting device in error state
        */
        if (deviceStateReplyStr == "sRA SCdevicestate 0"){
            std::cout << "Laser is busy \n";
        }
        else if (deviceStateReplyStr == "sRA SCdevicestate 1"){
            std::cout << "Laser is ready \n";
        }
        else if (deviceStateReplyStr == "sRA SCdevicestate 2"){
            std::cout << "Laser reports error state: " << deviceStateReplyStr << std::endl;        
        }
        else{
            std::cout << "Laser reports unknown devicestate: " << deviceStateReplyStr << std::endl;
        }

        /*
        * Start streaming 'LMDscandata'.
        */
        const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
        result = sendSOPASCommand(requestScanData, NULL);
        if (result != 0){
            std::cout << "SOPAS - Error starting to stream 'LMDscandata' \n"; 
            return ExitError;
        }

        return ExitSuccess;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::stop_scanner(){
        /*
         * Stop streaming measurements
         */
        const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
        int result = sendSOPASCommand(requestScanData0, NULL);
        if (result != 0){
            // use printf because we cannot use printf from the destructor
            printf("\nSOPAS - Error stopping streaming scan data!\n");
        }else{
            printf("\nSOPAS - Stopped streaming scan data.\n");
        }
      return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::init_device(){
        /*
        * Create and initialize a new LIBUSB session.
        */
        int result = libusb_init(&ctx_);
        if (result != 0){
            printf("LIBUSB - Initialization failed with the following error code: %i.", result);
            return ExitError;
        }

        /*
        * Set the verbosity level to 3 as suggested in the documentation.
        */
        libusb_set_debug(ctx_, 3);

        /*
        * Get a list of all SICK TIM3xx devices connected to the USB bus.
        *
        * As a shortcut, you can also use the LIBUSB function:
        * libusb_open_device_with_vid_pid(ctx, 0x19A2, 0x5001).
        */
        // int vendorID = 0x19A2; // SICK AG
        // int deviceID = 0x5001; // TIM3XX
        // numberOfDevices_ = getSOPASDeviceList(ctx_, vendorID, deviceID, &devices_);

        // /*
        // * If available, open the first SICK TIM3xx device.
        // */
        // if (numberOfDevices_ == 0){
        //     std::cout << "No SICK TiM devices connected! \n";
        //     return ExitError;
        // }else if (numberOfDevices_ <= deviceNumber_){
        //     printf("Device number %d too high, only %zu SICK TiM scanners connected", device_number_, numberOfDevices_);
        //     return ExitError;
        // }

        // /*
        // * Print out the SOPAS device information to the console.
        // */
        // printSOPASDeviceInformation(numberOfDevices_, devices_);

        /*
        * Open the device handle and detach all kernel drivers.
        */
        libusb_open(devices_[deviceNumber_], &deviceHandle_);
        if (deviceHandle_ == NULL){
            std::cout << "LIBUSB - Cannot open device (permission denied?); please read sick_tim/README.md \n";
            return ExitError;
        }
        else{
            printf("LIBUSB - Device opened");
        }

        if (libusb_kernel_driver_active(deviceHandle_, 0) == 1){
            std::cout << "LIBUSB - Kernel driver active \n";
            if (libusb_detach_kernel_driver(deviceHandle_, 0) == 0){
                std::cout << "LIBUSB - Kernel driver detached! \n";
            }
        }

        /*
        * Claim the interface 0
        */
        result = libusb_claim_interface(deviceHandle_, 0);
        if (result < 0){
            std::cout << "LIBUSB - Cannot claim interface \n";
            return ExitError;
        }else{
            std::cout << "LIBUSB - Claimed interface \n";
        }

        return ExitSuccess;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::loopOnce(){
        return 0;
           
    }
        
    
}
