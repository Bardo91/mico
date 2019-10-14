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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCK_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCK_H_

#include <vector>
#include <functional>
#include <unordered_map>
#include <string>

#include <any>
#include <mico/flow/Policy.h>

namespace mico{
    class OutPipe;

    class Block{
    public:
        static std::string name() {return "Unnammed";}

        // BASE METHODS
        virtual bool configure(std::unordered_map<std::string, std::string> _params) {};
        virtual std::vector<std::string> parameters(){ return {}; };

        std::unordered_map<std::string, OutPipe*> getPipes();

        void start();
        void stop();
        
        // void operator()(std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid);

        int nInputs();
        std::vector<std::string> inputTags();

        int nOutputs();
        std::vector<std::string> outputTags();

        Policy* getPolicy();

        void connect(std::string _pipeTag, Block& _otherBlock);

        void disconnect(std::string _pipeTag);

        // DYNAMIC CREATION METHODS
        // void registerCallback(std::function<void(std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid)> _callback);
        
        // void setPolicy(Policy* _pol);


        // void connect(OutPipe *_pipe, std::vector<std::string> _tags);

    protected:
        virtual void loopCallback() {};

    protected:
        Policy *iPolicy_ = nullptr;
        std::unordered_map<std::string, OutPipe*> opipes_;
        std::thread loopThread_;
        bool runLoop_ = false;
    };

}

#endif