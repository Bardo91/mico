#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    // std::shared_ptr<MapDatabase<pcl::PointXYZINormal>> mongoDatabase (new MapDatabase<pcl::PointXYZINormal>("map"));

    // if(!mongoDatabase->init()){
    //     return 0;
    // }
    // // fill db with dummy dataframes
    // for (int i = 0 ; i < 10 ; i++){
    //     std::shared_ptr<Dataframe<pcl::PointXYZINormal>> dataf(new Dataframe<pcl::PointXYZINormal>(i));
    //     Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    //     dataf->pose(pose);
    //     mongoDatabase->update(dataf);
    // }

    //mongoDatabase->printDb();
    
    // create array to update db
    // const auto elements = {1, 2, 3};
    // auto array_builder = bsoncxx::builder::basic::array{};
    // for (const auto& element : elements) {
    //     array_builder.append(element);
    // }

    // // update data if new id == 5
    // db["micoDatabase"].update_one(document{} << "id" << 5 << finalize,
    //                   document{} << "$set" << open_document
    //                   << "position" << open_array << array_builder << close_array 
    //                   << close_document << finalize);

    mongocxx::instance instance{};
    mongocxx::uri uri("mongodb://localhost:27017");;
    mongocxx::client conn{uri};

    auto db = conn["micoDatabase"];

    // fill db with dummy dataframes
    for (int i = 0 ; i < 10 ; i++){
        std::shared_ptr<Dataframe<pcl::PointXYZINormal>> dataf(new Dataframe<pcl::PointXYZINormal>(i));
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        dataf->pose(pose);

        auto doc = bsoncxx::builder::basic::document{};
        doc.append(kvp("id" , dataf->id()));
        doc.append(kvp("position", [&](sub_array _child) {
            auto dfPose = dataf->pose();
            for(unsigned i = 0; i < 4 ; i++)
                for(unsigned j = 0; j < 4 ; j++)
                    _child.append(dfPose(i,j));
        }));
        auto res = db["micoDatabase"].insert_one(doc.view()); 
    }
    
    // create array to update db
    const auto elements = {1, 2, 3};
    auto array_builder = bsoncxx::builder::basic::array{};
    for (const auto& element : elements) {
        array_builder.append(element);
    }

    // // update data if new id == 5
    // db["micoDatabase"].update_one(document{} << "id" << 5 << finalize,
    //                   document{} << "$set" << open_document
    //                   << "position" << open_array << array_builder << close_array 
    //                   << close_document << finalize);
    
    // print all db
	mongocxx::cursor cursor = db["micoDatabase"].find({});
	for(auto doc : cursor) {
	  std::cout << bsoncxx::to_json(doc) << "\n";
	}

	return 0;
}
