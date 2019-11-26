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
    mongoDatabase->restoreDatabase("/home/marrcogrova/.mico/tmp/database.json");
    
    // now need iterate over all collection
    
    bsoncxx::stdx::optional<bsoncxx::document::value> resultDocument =
            mongoDatabase->dbCollection().find_one(document{} << "id" << 3 << finalize);
    bsoncxx::document::view viewDocumentResult = resultDocument.value();

    Dataframe<pcl::PointXYZRGBNormal> dataf = mongoDatabase->createDataframe(viewDocumentResult);

    std::cout << "id: " << dataf.id() << " Cloud size: " << dataf.cloud()->size() << "\n";
    std::cout << "intrinsics coefficients: " << dataf.intrinsics() << "\n";

    cv::imshow("left camera",dataf.leftImage());
    cv::waitKey(0);

	return 0;
}
