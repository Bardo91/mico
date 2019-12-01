// //---------------------------------------------------------------------------------------------------------------------
// //  mico
// //---------------------------------------------------------------------------------------------------------------------
// //  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
// //---------------------------------------------------------------------------------------------------------------------
// //  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// //  and associated documentation files (the "Software"), to deal in the Software without restriction,
// //  including without limitation the rights to use, copy, modify, merge, publish, distribute,
// //  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// //  furnished to do so, subject to the following conditions:
// //
// //  The above copyright notice and this permission notice shall be included in all copies or substantial
// //  portions of the Software.
// //
// //  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
// //  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// //  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
// //  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// //  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// //---------------------------------------------------------------------------------------------------------------------

// #include <iostream>

// #include <ctime>

// #include <any>
// #include <opencv2/opencv.hpp>

// #include <csignal>

// #include <mico/flow/mico_flow.h>

// using namespace mico;
// using namespace flow;

// bool run = true;
// void signal_handler(int signal) {
//   if(signal == SIGINT){
//       run = false;
//   }
// }

int main(){

//     std::cout << "Creating Blocks" << std::endl;
//     StreamDataset stream;
//     if(!stream.configure({
//             {"color","/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/rgb/left_%d.png"},
//             {"depth","/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/depth/depth_%d.png"},
//             {"calibration","/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/CalibrationFile.xml"}
//         })){
//             std::cout << "Failed configuration of camera" << std::endl;
//             return -1;
//     }
//     BlockImageVisualizer imgVis;
//     BlockImageVisualizer imgVisDepth;

//     std::cout << "Connecting blocks" << std::endl;
//     stream.connect("color", imgVis);
//     stream.connect("depth", imgVisDepth);

//     // Start streaming
//     stream.start();
//     std::cout << "Started stream" << std::endl;
    
//     while(run){
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
    
//     std::cout << "Finishing" << std::endl;
//     stream.stop();    
    
}