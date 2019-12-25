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

#include <mico/flow/blocks/misc/BlockPython.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>
#include <chrono>
#include <iostream>

#include <Python.h>

namespace mico{
    BlockPython::BlockPython(){
        
        blockInterpreter_ = new QGroupBox("");
        blockInterpreterLayout_ = new QVBoxLayout();
        blockInterpreter_->setLayout(blockInterpreterLayout_);
        
        pythonEditor_ = new QTextEdit;
        highlighter_ = new PythonSyntaxHighlighter(pythonEditor_->document());
        blockInterpreterLayout_->addWidget(pythonEditor_);
        runButton_ = new QPushButton("play");
        blockInterpreterLayout_->addWidget(runButton_);
        
        // connect(runButton_, &QPushButton::clicked, this, [this]() {
        //         this->runPythonCode();
        //     });
    }



    void BlockPython::runPythonCode(){

        std::string pythonCode = pythonEditor_->toPlainText().toStdString();

        Py_Initialize();
        PyRun_SimpleString(pythonCode.c_str());
    
    }
}
