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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKPYTHON_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKPYTHON_H_

#include <flow/Block.h>

#include <QTextEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <mico/flow/blocks/misc/python/PythonSyntaxHighlighter.h>

#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/eigen.h>

namespace mico{
    class BlockPython: public flow::Block {
    public:
        static std::string name() {return "Python";}

        BlockPython();
        ~BlockPython();

        virtual QWidget * customWidget(){
            return blockInterpreter_;
        }

    private:
        void runPythonCode(flow::DataFlow _data, bool _useData);

        void encodeInput(pybind11::dict, flow::DataFlow _data, std::string _tag, std::string _typeTag);
        void flushPipe(pybind11::dict, std::string _tag, std::string _typeTag);

    private:
        bool idle_ = true;
        bool isReady_ = false;

        std::map<std::string, std::string> inputInfo_, outputInfo_;
        
        QGroupBox *blockInterpreter_;
        QVBoxLayout *blockInterpreterLayout_;
        QTextEdit * pythonEditor_;
        QPushButton * runButton_;
        PythonSyntaxHighlighter *highlighter_;
    };

}

#endif