#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    MapDatabase<pcl::PointXYZRGBNormal>::Ptr mongoDatabase (new MapDatabase<pcl::PointXYZRGBNormal>());
    
    if (!mongoDatabase->init(_argv[1],"load")){
        return 0;
    }
    std::map<int,Dataframe<pcl::PointXYZRGBNormal>::Ptr> dfs;
    if (mongoDatabase->restoreDatabaseFile("/home/grvc/.mico/tmp/database.json")){
        mongoDatabase->restoreDataframes(dfs);
    }
    
    for (auto &df : dfs){
        // check covisibility
        std::cout << df.second->id() << " - ";
        for (auto visibleDf : df.second->covisibility()){
            std::cout << visibleDf->id() << " ";
        }
        std::cout << std::endl;

        cv::imshow("ss",df.second->leftImage());
        cv::waitKey(0);

        std::cout << "pose: " << df.second->pose() << std::endl;
        std::cout << "Intrinsics: " << df.second->intrinsics() << std::endl;
        std::cout << "Coefficients: " << df.second->distCoeff() << std::endl;
        std::cout << "Cloud size: " << df.second->featureCloud()->size() << std::endl;
    }
	return 0;
}
