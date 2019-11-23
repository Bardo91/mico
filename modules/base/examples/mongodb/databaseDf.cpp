#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    MapDatabase<pcl::PointXYZINormal>::Ptr mongoDatabase (new MapDatabase<pcl::PointXYZINormal>("foo"));

    if(!mongoDatabase->init()){
        return 0;
    }
    // fill db with dummy dataframes
    for (int i = 0 ; i < 10 ; i++){
        std::shared_ptr<Dataframe<pcl::PointXYZINormal>> dataf(new Dataframe<pcl::PointXYZINormal>(i));
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        dataf->pose(pose);
        // cv::Mat img = cv::imread("/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/rgb/left_"+ std::to_string(i) +".png", 0);
        // dataf->leftImage(img);
        mongoDatabase->update(dataf);
    }  

    // mongoDatabase->printDb();

	return 0;
}
