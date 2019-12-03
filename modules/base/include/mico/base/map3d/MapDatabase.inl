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

        #ifdef USE_MONGO
            dbName_ = _databaseName;
            uri_ = mongocxx::uri("mongodb://localhost:27017");
            
            if (_mode == "save"){
                pathDbFolder_= "/home/marrcogrova/.mico/tmp";
                int stat = mkdir(pathDbFolder_.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
                if (stat == -1){
                    std::cout << "Error creating tmp folder \n";
                    return false;
                }
            }else if (_mode == "load"){
                
            }

            mongocxx::instance instance{};
            connClient_ = mongocxx::client{uri_};
            db_ = connClient_[dbName_]; 
            db_[dbName_].drop(); // clean db collection

            return true;
        #else
            return false;
        #endif
    }


    template<typename PointType_>
    inline bool MapDatabase<PointType_>::restoreDatabase(std::string _pathDatabase){
        #ifdef USE_MONGO
            std::ifstream file;
            file.open(_pathDatabase); 

            if (!file.is_open())
                return 0;
            
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
        #else
            return false;
        #endif
    }


    template<typename PointType_>
    inline bool MapDatabase<PointType_>::update(std::shared_ptr<mico::Dataframe<PointType_>> &_df){

        #ifdef USE_MONGO
            std::string dfFolder = pathDbFolder_ + "/dataframe_" + std::to_string(_df->id());
            int stat = mkdir(dfFolder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            if (stat == -1){
                std::cout << "Error creating dataframe folder \n";
                return false;
            }

            auto doc = bsoncxx::builder::basic::document{};
            doc.append(kvp("id" , _df->id()));

            doc.append(kvp("pose", [&](sub_array _child) {
                Eigen::Matrix4f dfPose = _df->pose();
                for(unsigned i = 0; i < _df->pose().rows() ; i++)
                    for(unsigned j = 0; j < _df->pose().cols() ; j++)
                        _child.append(dfPose(i,j));
            }));

            doc.append(kvp("intrinsics", [&](sub_array _child) {
                cv::Mat dfIntrinsics = _df->intrinsics();
                for(int i = 0; i < dfIntrinsics.rows ; i++)
                    for(int j = 0; j < dfIntrinsics.cols ; j++)
                        _child.append(dfIntrinsics.at<float>(i,j));
            }));

            doc.append(kvp("coefficients", [&](sub_array _child) {
                cv::Mat dfCoeff = _df->distCoeff();
                for(int i = 0; i < dfCoeff.rows ; i++)
                    for(int j = 0; j < dfCoeff.cols ; j++)
                        _child.append(dfCoeff.at<float>(i,j));
            }));

            cv::imwrite(dfFolder + "/left.png", _df->leftImage()); //666 image is gray
            doc.append(kvp("left_path" , dfFolder + "/left.png"));

            cv::imwrite(dfFolder + "/depth.png", _df->depthImage());
            doc.append(kvp("depth_path" , dfFolder + "/depth.png"));    

            pcl::io::savePCDFile(dfFolder + "/cloud.pcd", *_df->cloud(), true ); // true to use binary format
            doc.append(kvp("cloud_path" , dfFolder + "/cloud.pcd"));
            
            pcl::io::savePCDFile(dfFolder + "/featureCloud.pcd", *_df->featureCloud(), true );
            doc.append(kvp("featureCloud_path" , dfFolder + "/featureCloud.pcd"));

            doc.append(kvp("covisibility_ids", [&](sub_array _child) {
                for (auto visibleDf : _df->covisibility())
                _child.append(visibleDf->id());
            }));

            // add wordsReference_


            auto res = db_[dbName_].insert_one(doc.view());
        
            fileDatabase_.open(pathDbFolder_ + "/database.json" , std::ofstream::app); //append mode
            if (!fileDatabase_.is_open()){
                std::cout << "Error opening database json\n";
                return false;
            }
            fileDatabase_ << bsoncxx::to_json(doc) << "\n";
            fileDatabase_.close();

            return true;
        #else
            return false;
        #endif
    }

    #ifdef USE_MONGO
        std::vector<float> arrayView2Vector(bsoncxx::array::view _view){
            std::vector<float> viewVector;
            for (bsoncxx::array::element elem : _view) {
                float data = static_cast<float>(elem.get_double());
                viewVector.push_back(data);
            }
            return viewVector;
        }
    #endif
    
    #ifdef USE_MONGO
    template<typename PointType_>
    inline Dataframe<PointType_> MapDatabase<PointType_>::createDataframe(bsoncxx::document::view _doc ){
        int id = _doc["id"].get_value().get_int32().value ;

        Dataframe<PointType_> df(id);

        bsoncxx::array::view intrinsics = _doc["intrinsics"].get_value().get_array().value;
        std::vector<float> intrinsicsVector = arrayView2Vector(intrinsics);
        cv::Mat intrinsicsMat(intrinsicsVector);
        df.intrinsics(intrinsicsMat);

        bsoncxx::array::view coefficients = _doc["coefficients"].get_value().get_array().value;
        std::vector<float> coefficientsVector = arrayView2Vector(coefficients);
        cv::Mat coefficientsMat(coefficientsVector);
        df.distCoeff(coefficientsMat);

        auto pathLeft_doc = _doc["left_path"].get_value().get_utf8().value;
        std::string pathLeft = static_cast<std::string>(pathLeft_doc);
        cv::Mat left = cv::imread(pathLeft);
        df.leftImage(left);

        auto pathDepth_doc = _doc["depth_path"].get_value().get_utf8().value;
        std::string pathDepth = static_cast<std::string>(pathDepth_doc);
        cv::Mat depth = cv::imread(pathDepth);
        df.depthImage(depth);

        bsoncxx::array::view pose = _doc["pose"].get_value().get_array().value;
        std::vector<float> poseVector = arrayView2Vector(pose);
        Eigen::Matrix4f poseEigen = Eigen::Map<Eigen::Matrix<float, 4, 4> >(poseVector.data());
        df.pose( poseEigen );

        auto pathCloud_doc = _doc["cloud_path"].get_value().get_utf8().value;
        std::string pathCloud = static_cast<std::string>(pathCloud_doc);
        pcl::PointCloud<PointType_> cloud;
        pcl::io::loadPCDFile<PointType_>(pathCloud, cloud);
        df.cloud(cloud.makeShared());
        
        auto pathFeatureCloud_doc = _doc["featureCloud_path"].get_value().get_utf8().value;
        std::string pathFeatureCloud = static_cast<std::string>(pathFeatureCloud_doc);
        pcl::PointCloud<PointType_> featureCloud;
        pcl::io::loadPCDFile<PointType_>(pathFeatureCloud, featureCloud);
        df.featureCloud(featureCloud.makeShared());

        bsoncxx::array::view covisibility = _doc["covisibility_ids"].get_value().get_array().value;
        std::vector<float> covisibilityVector = arrayView2Vector(covisibility);
        std::vector<std::shared_ptr<Dataframe<PointType_>>> cov;
        for (auto idVisibleDf : covisibilityVector){
            // cov.push_back(); // hacer push back de los dataframes con los id que haya en el vector
        }

        

        return df;
    }
    #endif


    template<typename PointType_>
    inline bool MapDatabase<PointType_>::saveAllDatabase(){
        #ifdef USE_MONGO
            fileDatabase_.open(pathDbFolder_ + "/database.json"); 
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
        #else
            return false;
        #endif
    }


    template<typename PointType_>
    inline bool MapDatabase<PointType_>::printDatabase(){
        #ifdef USE_MONGO
            mongocxx::cursor cursor = db_[dbName_].find({});
            for(auto doc : cursor) {
            std::cout << bsoncxx::to_json(doc) << "\n";
            }
            return true;
        #else
            return false;
        #endif
    }
    
    #ifdef USE_MONGO
        template<typename PointType_>
        inline mongocxx::collection MapDatabase<PointType_>::dbCollection(){
            return db_[dbName_];
        }
    #endif
    
} // namespace mico 