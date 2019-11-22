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
using namespace mico;

class ConcatDf{
    public:
        int id() const{
            return id_;
        };
    private:
        int id_ = 2;
};

int main(int _argc, char** _argv) {
    // The mongocxx::instance constructor and destructor initialize and shut down the driver,
    // respectively. Therefore, a mongocxx::instance must be created before using the driver and
    // must remain alive for as long as the driver is in use.
    mongocxx::instance inst{};
    mongocxx::uri uri("mongodb://localhost:27017");;
    mongocxx::client conn{uri};

    auto db = conn["dataframeMap"];
	mongocxx::collection coll = db["dataframeMap"];

    Dataframe<pcl::PointXYZINormal>::Ptr dataf = Dataframe<pcl::PointXYZINormal>::Ptr(new Dataframe<pcl::PointXYZINormal>(10));

    for (int i = 0 ; i < 10 ; i++){
        auto builderDf = bsoncxx::builder::stream::document{};
            bsoncxx::document::value doc_value = builderDf
                    << "id" << i
                    << "position"    << open_array << 3 << 3 << 3 << close_array
                    << "orientation" << open_array << 3 << 3 << 3 << 1 << close_array
                << finalize;

        auto res = db["dataframeMap"].insert_one(std::move(doc_value));
    }

    
    db["dataframeMap"].update_one(document{} << "id" << 5 << finalize,
                      document{} << "$set" << open_document
                      << "position" << open_array << 100 << 1000 << 1000 << 1000 << close_array 
                      << close_document << finalize);
    
	mongocxx::cursor cursor = coll.find({});
	 for(auto doc : cursor) {
	   std::cout << bsoncxx::to_json(doc) << "\n";
	 }
	return 0;
}

bsoncxx::builder::stream::open_document_type& operator<< (bsoncxx::builder::stream::open_document_type& _os, ConcatDf& _conc){
    // _os << _conc.id;
    return _os;
}