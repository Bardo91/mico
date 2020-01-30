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

#include <QtWidgets>
#include <mico/flow/blocks/misc/InterfaceSelectorWidget.h>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <flow/DataFlow.h>

namespace mico{

    InterfaceElement::InterfaceElement(std::string _label){
        QHBoxLayout * layout = new QHBoxLayout;
        setLayout(layout);

        label_ =new QLineEdit(_label.c_str());
        typeList_ = new QComboBox;
        
        auto flowTypes = flow::TypeLog::registeredTypes();
        for(auto type:flowTypes){
            typeList_->addItem(type.c_str());
        }
        
        layout->addWidget(label_);
        layout->addWidget(typeList_);
    }


    InterfaceSelectorWidget::InterfaceSelectorWidget(std::string _title, bool _hasInputs, bool _hasOutputs, QWidget *parent): QDialog(parent) {
        hasInputs_ = _hasInputs;
        hasOutputs_ = _hasOutputs;
        
        twoColumnLayout_ = new QHBoxLayout;
        setLayout(twoColumnLayout_);

        // INPUTS
        if(hasInputs_){
            inputGroup_ = new QGroupBox("Inputs");
            QVBoxLayout *inGroupLayout = new QVBoxLayout;
            inputGroup_->setLayout(inGroupLayout);
            twoColumnLayout_->addWidget(inputGroup_);

            QHBoxLayout *countLayoutIn = new QHBoxLayout;
            inGroupLayout->addLayout(countLayoutIn);
            QLabel *countLabelIn = new QLabel("N. Interfaces");
            countLayoutIn->addWidget(countLabelIn); 
            countSelectorIn_ = new QSpinBox();
            countLayoutIn->addWidget(countSelectorIn_);
            connect(countSelectorIn_, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _n){ this->updateInterfacesIn(_n); });
            
            interfacesLayoutIn_ = new QVBoxLayout();
            inGroupLayout->addLayout(interfacesLayoutIn_);
        }

        // OUTPUTS
        if(hasOutputs_){
            outputGroup_ = new QGroupBox("Outputs");
            QVBoxLayout *outGroupLayout = new QVBoxLayout;
            outputGroup_->setLayout(outGroupLayout);
            twoColumnLayout_->addWidget(outputGroup_);

            QHBoxLayout *countLayoutOut = new QHBoxLayout;
            outGroupLayout->addLayout(countLayoutOut);
            QLabel *countLabelOut = new QLabel("N. Interfaces");
            countLayoutOut->addWidget(countLabelOut); 
            countSelectorOut_ = new QSpinBox();
            countLayoutOut->addWidget(countSelectorOut_);
            connect(countSelectorOut_, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _n){ this->updateInterfacesOut(_n); });
            
            interfacesLayoutOut_ = new QVBoxLayout();
            outGroupLayout->addLayout(interfacesLayoutOut_);
        }

        // OTHER
        setModal(true);
        setFocusPolicy(Qt::StrongFocus);
        setFocus();
        setWindowTitle(_title.c_str());
    }

    std::map<std::string, std::string> InterfaceSelectorWidget::getInterfaces(INTERFACE_TYPE _type) const{
        
        std::map<std::string, std::string> result;

        if(_type == INTERFACE_TYPE::INPUT){
            for(auto &interface: interfacesInput_){
                result[interface->label()] =  interface->type();
            }
        }else{
            for(auto &interface: interfacesOutput_){
                result[interface->label()] =  interface->type();
            }

        }
        return result;
    }


    void InterfaceSelectorWidget::updateInterfacesIn(int _nInterfaces){
        while(interfacesInput_.size() < _nInterfaces){
            interfacesInput_.push_back(new InterfaceElement("in_"+std::to_string(interfacesInput_.size())));
            interfacesLayoutIn_->addWidget(interfacesInput_.back());
            QWidget::connect(   interfacesInput_.back()->typeList_, 
                                QOverload<int>::of(&QComboBox::currentIndexChanged), 
                                [this](int _idx){this->callbackIfChangeInside_();});   // 666 More ugly code...;
        }
        
        while(interfacesInput_.size() > _nInterfaces){
            interfacesLayoutIn_->removeWidget(interfacesInput_.back());
            delete interfacesInput_.back(); // also removes it from upper layout
            interfacesInput_.pop_back();
        }

        // this->adjustSize();
        callbackIfChangeInside_();
    }


    void InterfaceSelectorWidget::updateInterfacesOut(int _nInterfaces){
        while(interfacesOutput_.size() < _nInterfaces){
            interfacesOutput_.push_back(new InterfaceElement("out_"+std::to_string(interfacesOutput_.size())));
            interfacesLayoutOut_->addWidget(interfacesOutput_.back());
            QWidget::connect(   interfacesOutput_.back()->typeList_, 
                                QOverload<int>::of(&QComboBox::currentIndexChanged), 
                                [this](int _idx){this->callbackIfChangeInside_();});   // 666 More ugly code...;
        }
        
        while(interfacesOutput_.size() > _nInterfaces){
            interfacesLayoutOut_->removeWidget(interfacesOutput_.back());
            delete interfacesOutput_.back(); // also removes it from upper layout
            interfacesOutput_.pop_back();
        }

        // this->adjustSize();
        callbackIfChangeInside_();
    }
}