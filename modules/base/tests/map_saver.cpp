//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <mico/base/map3d/Dataframe.h> 

using namespace mico;

TEST(map_saver, map_saver){
    // Create various dataframes
    Dataframe<pcl::PointXYZINormal>::Ptr dataf = Dataframe<pcl::PointXYZINormal>::Ptr(new Dataframe<pcl::PointXYZINormal>(10));

    std::map<int, Dataframe<pcl::PointXYZ>::Ptr> database;
    for(unsigned i = 1; i < 5; i++){
        database[i] = Dataframe<pcl::PointXYZ>::Ptr(new Dataframe<pcl::PointXYZ>(i));
    } 

    database[1]->appendCovisibility(database[3]); 

    database[2]->appendCovisibility(database[1]);
    database[2]->appendCovisibility(database[3]); 
    database[2]->appendCovisibility(database[4]); 

    database[3]->appendCovisibility(database[1]);
    database[3]->appendCovisibility(database[2]);
    database[3]->appendCovisibility(database[4]); 

    database[4]->appendCovisibility(database[2]);
    database[4]->appendCovisibility(database[3]);


    std::stringstream ss;

    std::string expectedId = "10\n";
    // ss << d;
    for (auto &df : database){
        ss << df.first << " - ";
        for (auto &seen :df.second->covisibility()){
            ss << seen << ",";
        }
        ss << "\n";
    }
    std::stringstream expectedResult;
    expectedResult << "1 - 3,\n" <<
                      "2 - 1,3,4,\n" <<
                      "3 - 1,2,4,\n" <<
                      "4 - 2,3,\n" ;

    
    // std::cout << "ID type: "<< typeid(dataf->id()).name() << std::endl;
    ASSERT_EQ(expectedResult.str() , ss.str());
}


std::ostream& operator<<(std::ostream& os, const std::map<int, Dataframe<pcl::PointXYZINormal>::Ptr>& _database){
    for (auto &df : _database){
            os << df.first << " "<< df.second;
    }
    return os;
}

// std::ostream& operator<<(std::ostream& os, const Dataframe<pcl::PointXYZINormal>::Ptr _df){
//     os << _df->id() << "\n";
//     return os;
// }

int main(int _argc, char **_argv)  {
    testing::InitGoogleTest(&_argc, _argv);
    return RUN_ALL_TESTS();
}