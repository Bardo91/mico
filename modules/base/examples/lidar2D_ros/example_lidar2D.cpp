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
#include <mico/base/LIDAR/LidarSICKTim517_parser.h>

int main(int _argc, char **_argv){

  ros::init(_argc, _argv, "mico_sickTim571_node");
  mico::LidarSICKTim571Parser* parser = new mico::LidarSICKTim571Parser();
      
  int device_number = 0;
      
  int result = mico::ExitError;
  while (ros::ok()){

    mico::LidarSICKTimHandlerUSB* handlerUsb = new mico::LidarSICKTimHandlerUSB(parser, device_number);
    result = handlerUsb->init();
    
    while(ros::ok() && (result == mico::ExitSuccess)){
      ros::spinOnce();
      result = handlerUsb->loopOnce();
    }
    delete handlerUsb;
     
    if (result == mico::ExitFatal)
      return result;
      
    if (ros::ok()){
      ros::Duration(1.0).sleep(); // Only attempt USB connections once per second
      // std::cout << "hei\n";
    }
  }

  delete parser;
  return result;
}