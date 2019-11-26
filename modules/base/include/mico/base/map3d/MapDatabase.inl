//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Marco Montes Grova (a.k.a marrcogrova)
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

namespace mico 
{   

    template<typename PointType_>
    inline MapDatabase<PointType_>::MapDatabase(){
    }

    template<typename PointType_>
    inline MapDatabase<PointType_>::~MapDatabase(){
    }
    
    template<typename PointType_>
    inline bool MapDatabase<PointType_>::init(std::string _databaseName, std::string _mode){

        dbName_ = _databaseName;
        uri_ = mongocxx::uri("mongodb://localhost:27017");
        
        mongocxx::instance instance{};
        connClient_ = mongocxx::client{uri_};
        db_ = connClient_[dbName_]; 
        
        if (_mode == "save"){
            pathDbFolder_= "/home/marrcogrova/.mico/tmp";
            int stat = mkdir(pathDbFolder_.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            if (stat == -1){
                std::cout << "Error creating tmp folder \n";
                return false;
            }

            db_[dbName_].drop(); // clean db collection
            return true;
        }else if (_mode == "load"){
            db_[dbName_].drop(); // clean db collection
            return true;
        }

    }

    template<typename PointType_>
    inline bool MapDatabase<PointType_>::restoreDatabase(std::string _pathDatabase){

        std::ifstream file;
        file.open(_pathDatabase); 

        if (!file.is_open()){
            return 0;
        }
        
        std::string line;
        std::vector<std::string> lines;
        while ( std::getline(file,line) ) 
            lines.push_back(line);
        file.close();

        std::vector<bsoncxx::document::value> vecDocs;
        for (unsigned int i=0 ; i<lines.size() - 1 ; i++){ 
            bsoncxx::document::value aux = bsoncxx::from_json(lines[i]);
            vecDocs.push_back(aux);
        }
        db_[dbName_].insert_many(vecDocs);

        return true;
    }

    template<typename PointType_>
    inline mongocxx::collection MapDatabase<PointType_>::dbCollection(){
        return db_[dbName_];
    }
    
    template<typename PointType_>
    inline bool MapDatabase<PointType_>::update(std::shared_ptr<mico::Dataframe<PointType_>> &_df){

        auto doc = bsoncxx::builder::basic::document{};
        doc.append(kvp("id" , _df->id()));
        doc.append(kvp("pose", [&](sub_array _child) {
            auto dfPose = _df->pose();
            for(unsigned i = 0; i < 4 ; i++)
                for(unsigned j = 0; j < 4 ; j++)
                    _child.append(dfPose(i,j));
        }));

        std::string dfFolder = pathDbFolder_ + "/dataframe_" + std::to_string(_df->id());
        int stat = mkdir(dfFolder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (stat == -1){
            std::cout << "Error creating dataframe folder \n";
            return false;
        }

        cv::imwrite(dfFolder + "/color.png", _df->leftImage()); //666 image is gray
        doc.append(kvp("left_path" , dfFolder + "/color.png"));    

        pcl::io::savePCDFile(dfFolder + "/cloud.pcd", *_df->cloud(), true ); // true to use binary format
        doc.append(kvp("cloud_path" , dfFolder + "/cloud.pcd"));
        
        // Save all df data

        auto res = db_[dbName_].insert_one(doc.view());
    
        fileDatabase_.open(pathDbFolder_ + "/database.json" , std::ofstream::app); //append mode
        if (!fileDatabase_.is_open()){
            std::cout << "Error opening database json\n";
            return false;
        }
        fileDatabase_ << bsoncxx::to_json(doc) << "\n";
        fileDatabase_.close();

        return true;
    }

    template<typename PointType_>
    inline Dataframe<PointType_> MapDatabase<PointType_>::createDataframe(bsoncxx::document::view _doc ){
        int id = _doc["id"].get_value().get_int32().value ;

        Dataframe<PointType_> df(id);

        bsoncxx::array::view pose = _doc["pose"].get_value().get_array().value;
        std::vector<float> poseVector;
        for (bsoncxx::array::element elem : pose) {
            float data = static_cast<float>(elem.get_double());
            poseVector.push_back(data);
        }
        Eigen::Matrix4f poseEigen = Eigen::Map<Eigen::Matrix<float, 4, 4> >(poseVector.data());
        df.pose( poseEigen );

        auto pathLeft_doc = _doc["left_path"].get_value().get_utf8().value;
        std::string pathLeft = static_cast<std::string>(pathLeft_doc);
        cv::Mat left = cv::imread(pathLeft);
        df.leftImage(left);

        auto pathCloud_doc = _doc["cloud_path"].get_value().get_utf8().value;
        std::string pathCloud = static_cast<std::string>(pathCloud_doc);
        pcl::PointCloud<PointType_> cloud;
        pcl::io::loadPCDFile<PointType_>(pathCloud, cloud );
        df.cloud(cloud.makeShared());
        
        return df;
    }

    template<typename PointType_>
    inline bool MapDatabase<PointType_>::saveAllDatabase(){
        fileDatabase_.open(pathDbFolder_ + "/database.json" , std::ofstream::app); 
        if (!fileDatabase_.is_open()){
            std::cout << "Error opening database json\n";
            return false;
        }

	    mongocxx::cursor cursor = db_[dbName_].find({});
	    for(auto doc : cursor) {
	      fileDatabase_ << bsoncxx::to_json(doc) << "\n";
	    }
        fileDatabase_.close();
        return true;
    }

    template<typename PointType_>
    inline bool MapDatabase<PointType_>::printDatabase(){
	    mongocxx::cursor cursor = db_[dbName_].find({});
	    for(auto doc : cursor) {
	      std::cout << bsoncxx::to_json(doc) << "\n";
	    }
        return true;
    }
    
} // namespace mico 