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

#include <mico/base/LIDAR/LidarSICKTimHandlerUSB.h>

namespace mico{
    LidarSICKTimHandlerUSB::LidarSICKTimHandlerUSB(LidarParser* _parser, int _deviceNumber){
        parser_ = _parser;
        deviceNumber_ = _deviceNumber;
        ctx_             = nullptr;
        numberOfDevices_ = 0;
        devices_         = nullptr;
        deviceHandle_   = nullptr;

        // init publisher handler.. eg. can be ROS publisher

    }

    //---------------------------------------------------------------------------------------------------------------------
    LidarSICKTimHandlerUSB::~LidarSICKTimHandlerUSB(){
        stop_scanner();
        close_device();
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::string LidarSICKTimHandlerUSB::replyToString(const std::vector<unsigned char> &_reply){
        std::string reply_str;
        for (std::vector<unsigned char>::const_iterator it = _reply.begin(); it != _reply.end(); it++){
            if (*it > 13){ // filter control characters for display
                reply_str.push_back(*it);
            }
        }
        return reply_str;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool LidarSICKTimHandlerUSB::isCompatibleDevice(const std::string _identStr){
        char device_string[7];
        int version_major = -1;
        int version_minor = -1;

        if (sscanf(_identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
                    &version_major, &version_minor) == 3
                    && strncmp("TiM3", device_string, 4) == 0
                    && version_major >= 2 && version_minor >= 50){
            std::cout << "This scanner model/firmware combination does not support ranging output! \n";
            printf("This is a %s, firmware version %d.%d \n", device_string, version_major, version_minor);

            return false;
        }

        return true;
    }
    
    //---------------------------------------------------------------------------------------------------------------------

    // Send a SOPAS command to the device and print out the response to the console.
    int LidarSICKTimHandlerUSB::sendSOPASCommand(const char* _request, std::vector<unsigned char> * _reply){
      if (deviceHandle_ == NULL) {
        this->error("LIBUSB","device not open");
        return ExitError;
      }

      int result = 0;
      unsigned char receiveBuffer[65536];

      
      // Write a SOPAS variable read request to the device.
      // printf("LIBUSB","Write data... %s",_request );

      int actual_length = 0;
      int requestLength = strlen(_request);
      result = libusb_bulk_transfer(deviceHandle_, (2 | LIBUSB_ENDPOINT_OUT), (unsigned char*)_request, requestLength,
                                    &actual_length, 0);
      if (result != 0 || actual_length != requestLength){
        this->error("LIBUSB","Write Error: " + std::to_string(result));
        return result;
      }

      // Read the SOPAS device response with the given timeout.
      result = libusb_bulk_transfer(deviceHandle_, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual_length, USB_TIMEOUT);
      if (result != 0){
        this->error("LIBUSB","Read Error: " + std::to_string(result));
        return result;
      }

      receiveBuffer[actual_length] = 0;
      // printf("LIBUSB - Read data...  %s", receiveBuffer);
      if(_reply) {
          _reply->clear();
          for(int i = 0; i < actual_length; i++) {
              _reply->push_back(receiveBuffer[i]);
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
            std::cout << "Failed to init device: " << result << std::endl;
        }

        return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::init_scanner(){

        //Read the SOPAS variable 'DeviceIdent' by index.
        const char requestDeviceIdent[] = "\x02sRI0\x03\0";
        std::vector<unsigned char> identReply;
        int result = sendSOPASCommand(requestDeviceIdent, &identReply);
        if (result != 0){
            this->error("SOPAS","Error reading variable 'DeviceIdent'.");
        }

        // Read the SOPAS variable 'SerialNumber' by name
        const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
        std::vector<unsigned char> serialReply;
        result = sendSOPASCommand(requestSerialNumber, &serialReply);
        if (result != 0){
            this->error("SOPAS","Error reading variable 'SerialNumber'.");
        }

        // set hardware ID based on DeviceIdent and SerialNumber
        std::string identStr = replyToString(identReply);
        std::string serialStr = replyToString(serialReply);

        if (!isCompatibleDevice(identStr))
            return ExitFatal;

        //Read the SOPAS variable 'FirmwareVersion' by name
        const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
        result = sendSOPASCommand(requestFirmwareVersion, NULL);
        if (result != 0){
            this->error("SOPAS","Error reading variable 'FirmwareVersion'.");
        }

        // Read Device State
        const char requestDeviceState[] = {"\x02sRN SCdevicestate\x03\0"};
        std::vector<unsigned char> deviceStateReply;
        result = sendSOPASCommand(requestDeviceState, &deviceStateReply);
        if (result != 0){
            this->error("SOPAS","Error reading variable 'deviceState'.");
        }
        std::string deviceStateReplyStr = replyToString(deviceStateReply);

        
        // Process device state, 0=Busy, 1=Ready, 2=Error
        // If configuration parameter is set, try resetting device in error state
        if (deviceStateReplyStr == "sRA SCdevicestate 0"){
            // this->warning("Laser is busy");
        }
        else if (deviceStateReplyStr == "sRA SCdevicestate 1"){
            // this->warning("Laser is ready");
        }
        else if (deviceStateReplyStr == "sRA SCdevicestate 2"){
            // this->warning("Laser reports error state: " + deviceStateReplyStr);      
        }
        else{
            // this->warning("Laser reports unknown devicestate: " + deviceStateReplyStr);
        }

        // Start streaming 'LMDscandata'.
        const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
        result = sendSOPASCommand(requestScanData, NULL);
        if (result != 0){
            this->error("SOPAS","Error starting to stream 'LMDscandata'.");
            return ExitError;
        }

        return ExitSuccess;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::stop_scanner(){
        
        // Stop streaming measurements
        const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
        int result = sendSOPASCommand(requestScanData0, NULL);
        if (result != 0){
            // use printf because we cannot use printf from the destructor
            this->warning("SOPAS","Error stopping streaming scan data!");
        }else{
            this->warning("SOPAS","Stopped streaming scan data.");
        }
      return result;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::init_device(){

        //Create and initialize a new LIBUSB session.
        int result = libusb_init(&ctx_);
        if (result != 0){
            this->error("LIBUSB","Initialization failed with the following error code: " + std::to_string(result));
            return ExitError;
        }
        
        // Set the verbosity level to 3 as suggested in the documentation.
        libusb_set_debug(ctx_, 3);

        // Get a list of all SICK TIM5xx devices connected to the USB bus.
        //   As a shortcut, you can also use the LIBUSB function:
        //   libusb_open_device_with_vid_pid(ctx, 0x19A2, 0x5001).
        int vendorID = 0x19A2; // SICK AG
        int deviceID = 0x5001; // TIM5XX
        numberOfDevices_ = getSOPASDeviceList(ctx_, vendorID, deviceID, &devices_);
        
        // If available, open the first SICK TIM5xx device
        if (numberOfDevices_ == 0){
            std::cout << "No SICK TiM devices connected! \n";
            return ExitError;
        }else if (numberOfDevices_ <= deviceNumber_){
            printf("Device number %d too high, only %zu SICK TiM scanners connected \n", deviceNumber_, numberOfDevices_);
            return ExitError;
        }

        //Print out the SOPAS device information to the console.
        // printSOPASDeviceInformation(numberOfDevices_, devices_);

        // Open the device handle and detach all kernel drivers.
        libusb_open(devices_[deviceNumber_], &deviceHandle_);
        if (deviceHandle_ == NULL){
            std::cout << "LIBUSB - Cannot open device (permission denied?); please read sick_tim/README.md \n";
            return ExitError;
        }
        else{
            // this->status("LIBUSB - Device opened");
        }

        if (libusb_kernel_driver_active(deviceHandle_, 0) == 1){
            std::cout << "LIBUSB - Kernel driver active \n";
            if (libusb_detach_kernel_driver(deviceHandle_, 0) == 0){
                std::cout << "LIBUSB - Kernel driver detached! \n";
            }
        }

        // Claim the interface 0
        result = libusb_claim_interface(deviceHandle_, 0);
        if (result < 0){
            this->error("LIBUSB","Cannot claim interface");
            return ExitError;
        }else{
            this->status("LIBUSB","Claimed interface");
        }

        return ExitSuccess;
    }

    //---------------------------------------------------------------------------------------------------------------------
    
    // Returns a list of USB devices currently attached to the system and matching the given vendorID and _productID.
    ssize_t LidarSICKTimHandlerUSB::getSOPASDeviceList(libusb_context *_ctx, uint16_t _vendorID, uint16_t _productID,
                                                libusb_device ***_list){
        
        libusb_device **resultDevices = NULL;
        ssize_t numberOfResultDevices = 0;
        libusb_device **devices;

        //Get a list of all USB devices connected.
        ssize_t numberOfDevices = libusb_get_device_list(_ctx, &devices);

        // Iterate through the list of the connected USB devices and search for devices with the given _vendorID and _productID.
        for (ssize_t i = 0; i < numberOfDevices; i++){
            struct libusb_device_descriptor desc;
            int result = libusb_get_device_descriptor(devices[i], &desc);
            if (result < 0){
                this->warning("LIBUSB","Failed to get device descriptor");
                continue;
            }

            if (desc.idVendor == _vendorID && desc.idProduct == 0x5001){ //666 with _productID
                
                // Add the matching device to the function result list and increase the device reference count.
                resultDevices = (libusb_device **)realloc(resultDevices, sizeof(libusb_device *) * (numberOfResultDevices + 2));
                if (resultDevices == NULL){
                    std::cout << "LIBUSB - Failed to allocate memory for the device result list. \n";
                }else{
                    resultDevices[numberOfResultDevices] = devices[i];
                    resultDevices[numberOfResultDevices + 1] = NULL;
                    libusb_ref_device(devices[i]);
                    numberOfResultDevices++;
                }
            }
        }

        // Free the list of the connected USB devices and decrease the device reference count.
        libusb_free_device_list(devices, 1);

        
        // Prepare the return values of the function.
        *_list = resultDevices;
        return numberOfResultDevices;
    }

    //---------------------------------------------------------------------------------------------------------------------
    
    // Free the list of devices obtained from the function 'getSOPASDeviceList'.
    void LidarSICKTimHandlerUSB::freeSOPASDeviceList(libusb_device **_list){
        if (!_list)
            return;

        int i = 0;
        struct libusb_device *dev;
        while ((dev = _list[i++]) != NULL){
            libusb_unref_device(dev);
        }
        free(_list);
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::get_datagram(unsigned char* _receiveBuffer, int _bufferSize, int* _actualLength){

        int result = libusb_bulk_transfer(deviceHandle_, (1 | LIBUSB_ENDPOINT_IN), _receiveBuffer, _bufferSize - 1, _actualLength,
                                        USB_TIMEOUT);   // read up to _bufferSize - 1 to leave space for \0
        if (result != 0){
            if (result == LIBUSB_ERROR_TIMEOUT){
                std::cout <<"LIBUSB - Read Error: LIBUSB_ERROR_TIMEOUT. \n";
                *_actualLength = 0;
                return ExitSuccess; // return success with size 0 to continue looping
            }else{
                this->error("LIBUSB","Read Error: "+ std::to_string(result));
                return result; // return failure to exit node
            }
        }

        _receiveBuffer[*_actualLength] = 0;
        return ExitSuccess;
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::loopOnce(){

        unsigned char receiveBuffer[65536];
        int actual_length = 0;
        // static unsigned int iteration_count = 0;

        int result = get_datagram(receiveBuffer, 65536, &actual_length);
        if (result != 0){
            printf("Read Error when getting datagram: %i \n",result);
            return ExitError; // return failure to exit node
        }
        if(actual_length <= 0){
            return ExitSuccess; // return success to continue looping
        }
        
        pcl::PointCloud<PointType_>::Ptr cloud2D(new pcl::PointCloud<PointType_>);
        // Datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
        char* buffer_pos = (char*)receiveBuffer;
        char *dstart, *dend;
        while( (dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)) )
        {
            size_t dlength = dend - dstart;
            *dend = '\0';
            dstart++;
            int success = parser_->parse_datagram(dstart, dlength, *cloud2D);
            if (success == ExitSuccess){
                
                parser_->cloud(cloud2D);

            }
            buffer_pos = dend + 1;
        }

        return ExitSuccess; // return success to continue looping
           
    }

    //---------------------------------------------------------------------------------------------------------------------
    int LidarSICKTimHandlerUSB::close_device(){
        int result = 0;
        if (deviceHandle_ != NULL){    
            // Release the interface
            result = libusb_release_interface(deviceHandle_, 0);
            if (result != 0)
                this->status("LIBUSB","Cannot Release Interface!");
            else
                this->status("LIBUSB","Released Interface!");

            //Close the device handle.
            libusb_close(deviceHandle_);
        }
        
        // Free the list of the USB devices.
        freeSOPASDeviceList(devices_);

        //Close the LIBUSB session.
        libusb_exit(ctx_);
        return result;
    }
        
    
}
