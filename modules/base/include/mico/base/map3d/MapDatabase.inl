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

namespace mico 
{   

    template<typename PointType_>
    inline MapDatabase<PointType_>::MapDatabase(std::string _databaseName){
        std::cout << "hi im mongo \n";
        dbName_ = _databaseName;
    }

    template<typename PointType_>
    inline MapDatabase<PointType_>::~MapDatabase(){
    }
    
    template<typename PointType_>
    inline bool MapDatabase<PointType_>::init(){
        mongocxx::instance instance{};
        mongocxx::uri uri("mongodb://localhost:27017");;
        mongocxx::client conn{uri};

        db_ = conn["map"];
        std::cout << "db initialized \n";
        return true;
    }

    template<typename PointType_>
    inline bool MapDatabase<PointType_>::update(std::shared_ptr<mico::Dataframe<PointType_>> &_df){
        
        // bsoncxx::stdx::optional<bsoncxx::document::value> maybe_result =
        //     db_["map"].find_one(document{} << "id" << 3 << finalize);
        // if(maybe_result) {
        //   std::cout << bsoncxx::to_json(*maybe_result) << "\n";
        // }

        auto doc = bsoncxx::builder::basic::document{};
        doc.append(kvp("id" , _df->id()));
        doc.append(kvp("pose", [&](sub_array _child) {
            auto dfPose = _df->pose();
            for(unsigned i = 0; i < 4 ; i++)
                for(unsigned j = 0; j < 4 ; j++)
                    _child.append(dfPose(i,j));
        }));
        auto res = db_["map"].insert_one(doc.view());  // 666 break. Collection??

        return true;
    }

    template<typename PointType_>
    inline bool MapDatabase<PointType_>::printDb(){

	    // mongocxx::cursor cursor = db_["map"].find({});
	    // for(auto doc : cursor) {
	    //   std::cout << bsoncxx::to_json(doc) << "\n";
	    // }

        return true;
    }

    
} // namespace mico 