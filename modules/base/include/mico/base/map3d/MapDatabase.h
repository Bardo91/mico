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

#ifndef MICO_MAP3D_MAPDATABASE_H_
#define MICO_MAP3D_MAPDATABASE_H_

#include <mico/base/map3d/Dataframe.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <sys/stat.h>

#ifdef USE_MONGO
#   include <bsoncxx/builder/basic/array.hpp>
#   include <bsoncxx/builder/stream/document.hpp>
#   include <bsoncxx/types.hpp>
#   include <bsoncxx/json.hpp>

#   include <mongocxx/client.hpp>
#   include <mongocxx/instance.hpp>
#   include <mongocxx/uri.hpp>
#endif

#ifdef USE_MONGO
    using bsoncxx::builder::stream::document;
    using bsoncxx::builder::stream::finalize;

    using bsoncxx::builder::basic::kvp;
    using bsoncxx::builder::basic::sub_array;

    using bsoncxx::builder::stream::open_document;
    using bsoncxx::builder::stream::close_document;
#endif

namespace mico {
    // use this struct?
    struct DatabaseTags{ 
        static std::string id_;
        static std::string pose_;
        static std::string leftPath_;
        static std::string cloudPath_;
    };

    template <typename PointType_>
    class MapDatabase{
        public:
            typedef std::shared_ptr<MapDatabase<PointType_>> Ptr;

            MapDatabase();
            ~MapDatabase();

            bool init(std::string _databaseName , std::string _mode);

            bool update(std::shared_ptr<mico::Dataframe<PointType_>> &_df); // using template only for it

            bool printDatabase();
            bool saveAllDatabase();

            bool restoreDatabase(std::string _pathDatabase);
            #ifdef USE_MONGO
                Dataframe<PointType_> createDataframe(bsoncxx::document::view _doc ); //666 Solve this specialization

            mongocxx::collection dbCollection();
            #endif

        private:
            std::string dbName_;

            std::string pathDbFolder_;
            std::ofstream fileDatabase_;

            #ifdef USE_MONGO
                mongocxx::uri uri_;
                mongocxx::client connClient_;
                mongocxx::database db_;
            #endif 
            
    };
} // namespace mico 

#include <mico/base/map3d/MapDatabase.inl>


#endif
