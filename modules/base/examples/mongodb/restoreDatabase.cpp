#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    MapDatabase<pcl::PointXYZRGBNormal>::Ptr mongoDatabase (new MapDatabase<pcl::PointXYZRGBNormal>());
    
    if (!mongoDatabase->init("reloaded" ,"load")){
        return 0;
    }
    std::map<int,Dataframe<pcl::PointXYZRGBNormal>::Ptr> dfs;
    if (mongoDatabase->restoreDatabaseFile("/home/grvc/.mico/tmp/database.json")){
        mongoDatabase->restoreDataframes(dfs);
    }
    
    // check covisibility
    for (auto &df : dfs){
        std::cout << df.second->id() << " - ";
        for (auto visibleDf : df.second->covisibility()){
            std::cout << visibleDf->id() << " ";
        }
        std::cout << std::endl;
    }

	return 0;
}
