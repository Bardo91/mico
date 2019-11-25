#include <iostream>
#include <chrono>

#include <mico/base/map3d/Dataframe.h> 

#include <mico/base/map3d/MapDatabase.h> 
#include <pcl/point_types.h>

using namespace mico;


int main(int _argc, char** _argv) {
    MapDatabase<pcl::PointXYZ>::Ptr mongoDatabase (new MapDatabase<pcl::PointXYZ>());

    std::ifstream file;
    file.open("/home/marrcogrova/Documents/datasets/databaseMongo/database.json"); 

    if (!file.is_open()){
        return 0;
    }
    std::cout << "file opened sucesfully\n";
    
    if (!mongoDatabase->init("reloaded")){
        return 0;
    }
    std::string line;
    std::vector<std::string> lines;
    while ( !std::getline(file,line).eof() ) 
        lines.push_back(line);
    file.close();

    std::vector<bsoncxx::document::value> vecDocs;
    for (int i=0 ; i<lines.size() - 1 ; i++){ 
        bsoncxx::document::value aux = bsoncxx::from_json(lines[i]);
        bsoncxx::document::view aux_view = aux.view();
        vecDocs.push_back(aux);
    }
    mongoDatabase->dbCollection().insert_many(vecDocs);
    mongoDatabase->printDatabase();
    
	return 0;
}
