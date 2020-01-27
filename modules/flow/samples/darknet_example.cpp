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

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

#include <csignal>

#include <mico/flow/mico_flow.h>

using namespace mico;


bool run = true;
void signal_handler(int signal) {
  if(signal == SIGINT){
      run = false;
  }
}

int main(){
    // #ifdef HAS_DARKNET

    // std::cout << "Creating Blocks" << std::endl;
    // StreamDataset stream;
    // BlockImageVisualizer imgVis;
    // BlockImageVisualizer imgVis2;
    // BlockImageVisualizer imgVisDepth;
    // BlockDarknet imgDetector;
    // BlockOdometryRGBD rgbdOdo;
    // BlockDatabaseMarkI db;
    // BlockDatabaseVisualizer dbVis;
    // BlockSceneVisualizer sceneVis;
    // BlockDataframeToPose df2Pose;

    // if(!stream.configure({
    //         {"color","/home/grvc/programming/datasets/rgbd_dataset_freiburg1_desk/changed images/rgb/left_%d.png"},
    //         {"depth","/home/grvc/programming/datasets/rgbd_dataset_freiburg1_desk/changed images/depth/depth_%d.png"},
    //         {"calibration","/home/grvc/programming/datasets/rgbd_dataset_freiburg1_desk/changed images/CalibrationFile.xml"}
    //     })){
    //         std::cout << "Failed configuration of camera" << std::endl;
    //         return -1;
    // }
    // std::cout << "Streamer configured" << std::endl;
    
    // if(!rgbdOdo.configure({
    //         {"calibration","/home/grvc/programming/datasets/rgbd_dataset_freiburg1_desk/CalibrationFile.xml"}
    //     })){
    //         std::cout << "Failed configuration of odometry" << std::endl;
    //         return -1;
    // }
    // std::cout << "Odometry configured" << std::endl;

    // if(!db.configure({
    //         {"vocabulary","/home/grvc/programming/datasets/vocabulary_dbow2_livingroom_orb_k6L4.xml"},
    //         {"clusterComparison","0.8"}
    //     })){
    //         std::cout << "Failed configuration of database" << std::endl;
    //         return -1;
    // }
    // std::cout << "database configured" << std::endl;

    // if(!imgDetector.configure({
    //         {"cfg","cfg"},
    //         {"weights","weights"},
    //         {"confidence_treshold","0.7"}
    //     })){
    //         std::cout << "Failed configuration of detector" << std::endl;
    //         return -1;
    // }
    // std::cout << "Detector configured" << std::endl;

    // if(!dbVis.configure({
    //         {"cs_scale","1"}
    //     })){
    //         std::cout << "Failed configuration of database visualizer" << std::endl;
    //         return -1;
    // }
    // std::cout << "Database visualizer configured" << std::endl;

    // if(!sceneVis.configure({
    //         {"voxel_size","1"},
    //         {"use_octree","false"},
    //         {"octree_depth","4"}
    //     })){
    //         std::cout << "Failed configuration of sceneVis visualizer" << std::endl;
    //         return -1;
    // }
    // std::cout << "sceneVis visualizer configured" << std::endl;
    // std::cout << "Connecting blocks" << std::endl;

    // stream.connect("color", imgVis);
    // //stream.connect("depth", imgVisDepth);
    // stream.connect("color", rgbdOdo);
    // stream.connect("depth", rgbdOdo);
    // stream.connect("cloud", rgbdOdo);
    // rgbdOdo.connect("dataframe", db);
    // //imgDetector.connect("color", imgVis2);
    // //imgDetector.connect("v_entity", dbVis);
    // db.connect("dataframe", rgbdOdo);
    // //db.connect("dataframe", imgDetector);
    // db.connect("dataframe", df2Pose);
    // //db.connect("dataframe", dbVis);
    // //df2Pose.connect("pose", dbVis);
    
    // db.connect("dataframe", sceneVis);
    // df2Pose.connect("pose", sceneVis);

    // // Start streaming
    // stream.start();
    // std::cout << "Started stream" << std::endl;
    
    // while(run){
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    
    // std::cout << "Finishing" << std::endl;
    // stream.stop();    
    
    // #endif
}
