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
    for (int i = 1 ; i < 100 ; i++){
        std::shared_ptr<Dataframe<pcl::PointXYZRGBNormal>> dataf(new Dataframe<pcl::PointXYZRGBNormal>(i));
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        dataf->pose(pose);

        cv::Mat imgL = cv::imread("/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/rgb/left_"+ std::to_string(i) +".png", 0);
        dataf->leftImage(imgL);
        cv::Mat imgD = cv::imread("/home/marrcogrova/Documents/datasets/rgbd_dataset_freiburg1_room/depth/depth_"+ std::to_string(i) +".png", 0);
        dataf->depthImage(imgD);
        cv::Mat intrinsics = (cv::Mat_<float>(3,3) << 696.262741 , 0.0 , 664.58279 , 0.0 , 696.715205 , 331.19961 , 0.0 , 0.0 , 1.0);
        dataf->intrinsics(intrinsics);
        cv::Mat coefficients = (cv::Mat_<float>(5,1) << -0.174138, 0.025148, -2.3e-05, 0.001127, 0.0);
        dataf->distCoeff(coefficients);

        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
        cloud.resize(5);
        for (unsigned int j=0 ; j< 5 ; j++){
            pcl::PointXYZRGBNormal p;
            p.x = 0; p.y = 0; p.z = 0;
            cloud.push_back(p);
        }
        dataf->cloud(cloud.makeShared());

        pcl::PointCloud<pcl::PointXYZRGBNormal> ftrsCloud;
        cloud.resize(5);
        for (unsigned int j=0 ; j< 5 ; j++){
            pcl::PointXYZRGBNormal p;
            p.x = 10; p.y = 10; p.z = 10;
            ftrsCloud.push_back(p);
        }
        dataf->featureCloud(ftrsCloud.makeShared());
        
        mongoDatabase->update(dataf);
    }
    mongoDatabase->saveAllDatabase();
    // mongoDatabase->printDatabase();

	return 0;
}
