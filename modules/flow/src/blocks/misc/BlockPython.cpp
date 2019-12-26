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

#include <QtWidgets>
#include <QPushButton>
#include <mico/flow/blocks/misc/InterfaceSelectorWidget.h>

namespace mico{
    BlockPython::BlockPython(){
        // Instantiate outputs
        InterfaceSelectorWidget selectorOutputs("Outputs");
        selectorOutputs.exec();

        outputInfo_ = selectorOutputs.getInterfaces();
        for(auto &output: outputInfo_){
            createPipe(output.first, output.second);
        }

        // Instantiate Inputs
        InterfaceSelectorWidget selectorInputs("Inputs");
        selectorInputs.exec();

        inputInfo_ = selectorInputs.getInterfaces();
        if(inputInfo_.size() > 0){
            createPolicy(inputInfo_);

            // registerCallback(, [&](flow::DataFlow _data){
            // });
        }


        blockInterpreter_ = new QGroupBox("");
        blockInterpreterLayout_ = new QVBoxLayout();
        blockInterpreter_->setLayout(blockInterpreterLayout_);
        
        pythonEditor_ = new QTextEdit;
        highlighter_ = new PythonSyntaxHighlighter(pythonEditor_->document());
        blockInterpreterLayout_->addWidget(pythonEditor_);
        runButton_ = new QPushButton("play");
        blockInterpreterLayout_->addWidget(runButton_);
        
        QWidget::connect(runButton_, &QPushButton::clicked, [this]() {
                flow::DataFlow data({}, [](flow::DataFlow _data){});
                this->runPythonCode(data);
            });
    }

    void replaceAll(std::string& str, const std::string& from, const std::string& to) {
        if(from.empty())
            return;
        size_t start_pos = 0;
        while((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }

    void BlockPython::runPythonCode(flow::DataFlow _data){

        std::string pythonCode = pythonEditor_->toPlainText().toStdString();

        replaceAll(pythonCode, "\n", "\n\t");
        pythonCode =    "def micoFlowFunction(inputMap):\n\t" + 
                        "outputMap = {}:\n\t" + 
                        pythonCode + 
                        "\n\t return outputMap";

        std::cout << pythonCode << std::endl;

        PyObject *pModule = PyImport_Import(pName);
        PyObject *pFunc = PyObject_GetAttrString(pModule, "micoFlowFunction");
        if (pFunc && PyCallable_Check(pFunc)) {
            PyObject = pArgs = PyTuple_New(inputInfo_.size());
            // Encode inputs

            PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
        
            // Encode outputs

        }
        Py_XDECREF(pFunc);
        
        // Py_Initialize();
        // PyRun_SimpleString(pythonCode.c_str());
    
    }
}
