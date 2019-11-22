#include <iostream>
#include <chrono>

#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

#include <mico/base/map3d/Dataframe.h> 
#include <pcl/point_types.h>

using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::open_document;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::finalize;

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::sub_array;

using namespace mico;

class ConcatDf{
    public:
        ConcatDf(int _id){
            id_ = _id;
            position_ = {1.0 , 10.0 , 100.0};
        };
        int id() const{
            return id_;
        };
        std::vector<float> position() const{
            return position_;
        };
    private:
        int id_;
        std::vector<float> position_;
};

int main(int _argc, char** _argv) {
    mongocxx::instance instance{};
    mongocxx::uri uri("mongodb://localhost:27017");;
    mongocxx::client conn{uri};

    auto db = conn["micoDatabase"];
	// mongocxx::collection coll = db["micoDatabase"];

    // fill db with dummy dataframes
    for (int i = 0 ; i < 10 ; i++){
        Dataframe<pcl::PointXYZINormal> dataf = Dataframe<pcl::PointXYZINormal>(i);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        dataf.pose(pose);

        auto doc = bsoncxx::builder::basic::document{};
        doc.append(kvp("id" , dataf.id()));
        doc.append(kvp("position", [&](sub_array _child) {
            std::vector<float> pos(dataf.position().data(), dataf.position().data() + dataf.position().size());
            for (const auto& element : pos ) {
                _child.append(element);
            }
        }));
        auto res = db["micoDatabase"].insert_one(doc.view()); 
    }
    
    // create array to update db
    const auto elements = {1, 2, 3};
    auto array_builder = bsoncxx::builder::basic::array{};
    for (const auto& element : elements) {
        array_builder.append(element);
    }

    // update data if new id == 5
    db["micoDatabase"].update_one(document{} << "id" << 5 << finalize,
                      document{} << "$set" << open_document
                      << "position" << open_array << array_builder << close_array 
                      << close_document << finalize);
    
    // print all db
	mongocxx::cursor cursor = db["micoDatabase"].find({});
	for(auto doc : cursor) {
	  std::cout << bsoncxx::to_json(doc) << "\n";
	}

	return 0;
}
