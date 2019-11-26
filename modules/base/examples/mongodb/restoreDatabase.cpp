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
    mongoDatabase->restoreDatabase("/home/marrcogrova/Documents/datasets/databaseMongo/database.json");
    
    // now need iterate over all collection
    
    bsoncxx::stdx::optional<bsoncxx::document::value> resultDocument =
            mongoDatabase->dbCollection().find_one(document{} << "id" << 3 << finalize);
    bsoncxx::document::view viewDocumentResult = resultDocument.value();

    Dataframe<pcl::PointXYZRGBNormal> dataf = mongoDatabase->createDataframe(viewDocumentResult);

    cv::imshow("left camera",dataf.leftImage());
    cv::waitKey(0);

	return 0;
}
