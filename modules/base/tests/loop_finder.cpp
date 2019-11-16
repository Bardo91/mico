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

#include <mico/base/map3d/LoopClosureDetector.h>
#include <gtest/gtest.h>
#include <pcl/point_types.h>

using namespace mico;
using namespace pcl;

template <DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
class LoopClosureDetectorDummy: public LoopClosureDetector<DebugLevel_, OutInterface_>{
public:
    /// Initialize loop closure detector with specific data for DLoopDetector by Dorian
    virtual bool init(cjson::Json &_config){}

    /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
    virtual LoopResult appendCluster(cv::Mat &_image, int _id){}

    /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
    virtual LoopResult appendCluster(const std::vector<cv::KeyPoint> &_kps, const cv::Mat &_descriptors, int _id){}
};


TEST(loop_finder, loop_finder)  {
    // Create various dataframes
    std::map<int, Dataframe<PointXYZ>::Ptr> database;
    for(unsigned i = 1; i < 8; i++){
        database[i] = Dataframe<PointXYZ>::Ptr(new Dataframe<PointXYZ>(i));
    } 

    // Append covisibility
    database[1]->appendCovisibility(database[2]); database[2]->appendCovisibility(database[1]);
    database[1]->appendCovisibility(database[3]); database[3]->appendCovisibility(database[1]);

    database[2]->appendCovisibility(database[3]); database[3]->appendCovisibility(database[2]);
    database[2]->appendCovisibility(database[4]); database[4]->appendCovisibility(database[2]);

    database[3]->appendCovisibility(database[4]); database[4]->appendCovisibility(database[3]);
    database[3]->appendCovisibility(database[5]); database[5]->appendCovisibility(database[3]);
    database[3]->appendCovisibility(database[6]); database[6]->appendCovisibility(database[3]);

    database[4]->appendCovisibility(database[5]); database[5]->appendCovisibility(database[4]);

    database[5]->appendCovisibility(database[6]); database[6]->appendCovisibility(database[5]);
    database[5]->appendCovisibility(database[7]); database[7]->appendCovisibility(database[5]);

    database[6]->appendCovisibility(database[7]); database[7]->appendCovisibility(database[6]);

    LoopClosureDetectorDummy<> lcd;

    std::vector<Dataframe<PointXYZ>::Ptr> result = lcd.findPath<PointXYZ>(database[1], database[7]);

    std::vector<Dataframe<PointXYZ>::Ptr> expectedResult = {
        database[1], 
        database[3],
        database[5],
        database[7]
    };

    ASSERT_EQ(result.size(), expectedResult.size());

    for(unsigned i = 0; i < result.size();i++){
        ASSERT_EQ(result[i]->id(), expectedResult[i]->id());
    }

}

 
int main(int _argc, char **_argv)  {
    testing::InitGoogleTest(&_argc, _argv);
    return RUN_ALL_TESTS();
}