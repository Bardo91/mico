//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKQUEUER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKQUEUER_H_

#include <flow/Block.h>
#include <mico/base/map3d/OdometryRgbd.h>
#include <deque>


namespace mico{

    template<typename Trait_>
    class BlockQueuer: public flow::Block{
    public:
        virtual std::string name() const override {return Trait_::Name_;}

        BlockQueuer(){
            createPipe(Trait_::OutputName_, Trait_::OutputType_);
            createPolicy({{{Trait_::InputName_, Trait_::InputType_}}});
            registerCallback({Trait_::InputName_}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        typename Trait_::Type_ data = _data.get<typename Trait_::Type_>(Trait_::InputName_);
                                        queue_.push_back(data);
                                        if(queue_.size() > size_){
                                            queue_.pop_front();
                                            if(strideCounter_ % stride_ == 0){
                                                getPipe(Trait_::OutputName_)->flush(std::vector<typename Trait_::Type_>({queue_.begin(), queue_.end()}));
                                                strideCounter_ = 0;
                                            }
                                            strideCounter_++;
                                        }
                                        idle_ = true;
                                    }
                                }
            );
        }
        ~BlockQueuer(){ }

        bool configure(std::unordered_map<std::string, std::string> _params) override{
            size_ = atoi(_params["queue_size"].c_str());
            stride_ = atoi(_params["stride"].c_str());
            return true;
        }

        std::vector<std::string> parameters() override{
            return {"queue_size", "stride"};
        }

        std::string description() const override {return    "Block that takes an input stream of data and places it into an array of given size."
                                                            "The queue is flushed by the given stride size.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};
    
    private:
        std::deque<typename Trait_::Type_> queue_;
        unsigned int size_ = 1;
        int stride_ = 1;
        int strideCounter_ = 0;
        bool idle_ = true;
    };


    //-----------------------------------------------------------------------------------------------------------------
    struct QueuerTraitClusterframes{
        constexpr static const char * Name_ = "Queuer Dataframes";
        constexpr static const char * OutputType_ = "v-dataframe";
        constexpr static const char * OutputName_ = "Vec dataframes";
        constexpr static const char * InputType_ = "dataframe";
        constexpr static const char * InputName_ = "dataframe";
        typedef std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> Type_;
    };

    //-----------------------------------------------------------------------------------------------------------------
    struct QueuerTraitColor{
        constexpr static const char * Name_ = "Queuer Color Images";
        constexpr static const char * OutputType_ = "v-image";
        constexpr static const char * OutputName_ = "Vec images";
        constexpr static const char * InputType_ = "image";
        constexpr static const char * InputName_ = "image";
        typedef cv::Mat Type_;
    };

}

#endif