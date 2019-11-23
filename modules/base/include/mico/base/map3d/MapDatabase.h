//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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

#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

#include <opencv2/opencv.hpp>

using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::sub_array;

namespace mico {

    template <typename PointType_>
    class MapDatabase{
        public:
            typedef std::shared_ptr<MapDatabase<PointType_>> Ptr;

            MapDatabase(std::string _databaseName);
            MapDatabase(std::string _databaseName , std::string _uri);
            ~MapDatabase();

            bool init();

            bool update(std::shared_ptr<mico::Dataframe<PointType_>> &_df);

            bool printDb();

        private:
            mongocxx::uri uri_;
            mongocxx::client connClient_;
            mongocxx::database db_; 
            std::string dbName_;

            // std::string pathFolder_= "/home/marrcorova/tmp";
            
    };
} // namespace mico 

#include <mico/base/map3d/MapDatabase.inl>


#endif
