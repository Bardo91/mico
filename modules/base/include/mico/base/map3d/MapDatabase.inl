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

namespace mico{   

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
            uri_ = mongocxx::uri{}; //default connect to a server localhost:27017
            
            if (_mode == "save"){
                pathDbFolder_= "/home/grvc/.mico/tmp";
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
            std::cout << "Must set USE_MONGO to ON \n";
            return false;
        #endif
    }


    template<typename PointType_>
    inline bool MapDatabase<PointType_>::restoreDatabaseFile(std::string _pathDatabase){
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
            for (unsigned int i=0 ; i<lines.size() ; i++){ // 666
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
    inline bool MapDatabase<PointType_>::restoreDataframes(std::map<int , std::shared_ptr<Dataframe<PointType_>>> &_dfs){
        
        // iterate over all collection
        mongocxx::cursor cursor = dbCollection().find({});
        for(auto doc : cursor) {
            Dataframe<pcl::PointXYZRGBNormal>::Ptr dataf = createDataframe(doc);
            _dfs[dataf->id()] = dataf;
        }
        
        // iterate over all dfs to fill covisibility
        for (auto dfIt=_dfs.begin(); dfIt!=_dfs.end(); ++dfIt){

            auto sucess = covisibilityDb_.find(dfIt->first);
            if (sucess != covisibilityDb_.end()){
                // std::cout << sucess->first << " - ";
                for (auto visibleId : sucess->second){
                    dfIt->second->appendCovisibility(_dfs[visibleId]); 
                    // std::cout << " " << visibleId;
                }
                // std::cout << std::endl;
            }else{
                return false;
            }
        }

        return true;
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
                        _child.append(std::to_string(dfPose(i,j)));
            }));

            doc.append(kvp("intrinsics", [&](sub_array _child) {
                cv::Mat dfIntrinsics = _df->intrinsics();
                for(int i = 0; i < dfIntrinsics.rows ; i++)
                    for(int j = 0; j < dfIntrinsics.cols ; j++) 
                        _child.append(std::to_string(dfIntrinsics.at<float>(i,j)));
            }));

            doc.append(kvp("coefficients", [&](sub_array _child) {
                cv::Mat dfCoeff = _df->distCoeff();
                for(int i = 0; i < dfCoeff.rows ; i++)
                    for(int j = 0; j < dfCoeff.cols ; j++)
                        _child.append(std::to_string(dfCoeff.at<float>(i,j)));
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
            // bsoncxx::stdx::optional<bsoncxx::document::value> addedSuccess =
            //     db_[dbName_].find_one(document{} << "id" << _df->id() << finalize);
            // if(addedSuccess)
            //     fileDatabase_ << bsoncxx::to_json(*addedSuccess) << "\n";
            
            fileDatabase_ << bsoncxx::to_json(doc) << "\n";
            fileDatabase_.close();

            return true;
        #else
            return false;
        #endif
    }

    #ifdef USE_MONGO
    template<typename PointType_>
    inline std::vector<float> MapDatabase<PointType_>::arrayView2Vectorf(bsoncxx::array::view _view){
        std::vector<float> viewVector;
        for (bsoncxx::array::element elem : _view) {
            // Save numbers as string to avoid comma separated because breaks json
            auto data_cod = elem.get_utf8().value;
            std::string data_str = static_cast<std::string>(data_cod);
            std::size_t pos=0;
            while (pos < data_str.size())
                if ((pos = data_str.find_first_of (',',pos)) != std::string::npos)
                    data_str[pos] = '.';

            float data = std::stof(data_str);
            viewVector.push_back(data);
        }
        return viewVector;
    }
    #endif
    
    #ifdef USE_MONGO
    template<typename PointType_>
    inline std::shared_ptr<Dataframe<PointType_>> MapDatabase<PointType_>::createDataframe(bsoncxx::document::view _doc ){
        int id = _doc["id"].get_value().get_int32().value;

        std::shared_ptr<Dataframe<PointType_>> df (new Dataframe<PointType_>(id));

        bsoncxx::array::view intrinsics = _doc["intrinsics"].get_value().get_array().value;
        std::vector<float> intrinsicsVector = arrayView2Vectorf(intrinsics);
        cv::Mat intrinsicsMat(1,intrinsicsVector.size(), CV_32FC1);
        memcpy(intrinsicsMat.data,intrinsicsVector.data(),intrinsicsVector.size()*sizeof(float));
        df->intrinsics(intrinsicsMat);

        bsoncxx::array::view coefficients = _doc["coefficients"].get_value().get_array().value;
        std::vector<float> coefficientsVector = arrayView2Vectorf(coefficients);
        cv::Mat coefficientsMat(1,coefficientsVector.size(), CV_32FC1);
        memcpy(coefficientsMat.data,coefficientsVector.data(),coefficientsVector.size()*sizeof(float));
        df->distCoeff(coefficientsMat);

        auto pathLeft_doc = _doc["left_path"].get_value().get_utf8().value;
        std::string pathLeft = static_cast<std::string>(pathLeft_doc);
        cv::Mat left = cv::imread(pathLeft);
        df->leftImage(left);

        auto pathDepth_doc = _doc["depth_path"].get_value().get_utf8().value;
        std::string pathDepth = static_cast<std::string>(pathDepth_doc);
        cv::Mat depth = cv::imread(pathDepth);
        df->depthImage(depth);

        bsoncxx::array::view pose = _doc["pose"].get_value().get_array().value;
        std::vector<float> poseVector = arrayView2Vectorf(pose);
        Eigen::Matrix4f poseEigen = Eigen::Map<Eigen::Matrix<float, 4, 4> >(poseVector.data());
        df->pose( poseEigen );

        auto pathCloud_doc = _doc["cloud_path"].get_value().get_utf8().value;
        std::string pathCloud = static_cast<std::string>(pathCloud_doc);
        pcl::PointCloud<PointType_> cloud;
        pcl::io::loadPCDFile<PointType_>(pathCloud, cloud);
        df->cloud(cloud.makeShared());
        
        auto pathFeatureCloud_doc = _doc["featureCloud_path"].get_value().get_utf8().value;
        std::string pathFeatureCloud = static_cast<std::string>(pathFeatureCloud_doc);
        pcl::PointCloud<PointType_> featureCloud;
        pcl::io::loadPCDFile<PointType_>(pathFeatureCloud, featureCloud);
        df->featureCloud(featureCloud.makeShared());

        bsoncxx::array::view covisibility = _doc["covisibility_ids"].get_value().get_array().value;
        std::vector<int> covVector;
        for (bsoncxx::array::element elem : covisibility) {
            int visibleId = static_cast<int>(elem.get_int32());
            covVector.push_back(visibleId);
        }

        covisibilityDb_.insert( std::pair<int,std::vector<int>>(df->id() ,covVector) );

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