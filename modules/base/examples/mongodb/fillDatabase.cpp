#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    MapDatabase<pcl::PointXYZRGBNormal>::Ptr mongoDatabase (new MapDatabase<pcl::PointXYZRGBNormal>());

    if(!mongoDatabase->init("foo" , "save")){
        return 0;
    }
    // fill db with dummy dataframes
    for (int i = 0 ; i < 10 ; i++){
        std::shared_ptr<Dataframe<pcl::PointXYZRGBNormal>> dataf(new Dataframe<pcl::PointXYZRGBNormal>(i));
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        dataf->pose(pose);

        cv::Mat img = cv::imread("/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/rgb/left_"+ std::to_string(i) +".png", 0);
        dataf->leftImage(img);
        
        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
        cloud.resize(5);
        for (unsigned int j=0 ; j< 5 ; j++){
            pcl::PointXYZRGBNormal p;
            p.x = 0; p.y = 0; p.z = 0;
            cloud.push_back(p);
        }
        dataf->cloud(cloud.makeShared());
        mongoDatabase->update(dataf);
    }
    mongoDatabase->saveAllDatabase();

	return 0;
}
