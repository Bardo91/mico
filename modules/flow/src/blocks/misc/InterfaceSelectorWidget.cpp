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

    InterfaceElement::InterfaceElement(){
        QHBoxLayout * layout = new QHBoxLayout;
        setLayout(layout);

        label_ =new QLineEdit("label");
        typeList_ = new QComboBox;
        
        auto flowTypes = flow::TypeLog::registeredTypes();
        for(auto type:flowTypes){
            typeList_->addItem(type.c_str());
        }
        
        layout->addWidget(label_);
        layout->addWidget(typeList_);
    }


    InterfaceSelectorWidget::InterfaceSelectorWidget(std::string _title, QWidget *parent): QDialog(parent) {
        mainLayout_ = new QVBoxLayout;
        setLayout(mainLayout_);

        QHBoxLayout *countLayout = new QHBoxLayout;
        mainLayout_->addLayout(countLayout);
        QLabel *countLabel = new QLabel("N. Interfaces");
        countLayout->addWidget(countLabel); 
        countSelector_ = new QSpinBox();
        countLayout->addWidget(countSelector_);
        connect(countSelector_, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _n){ this->updateInterfaces(_n); });
        // connect(countSelector_, &QSpinBox::valueChanged, this, &InterfaceSelectorWidget::updateInterfaces);

        interfacesLayout_ = new QVBoxLayout();
        mainLayout_->addLayout(interfacesLayout_);

        setModal(true);
        setFocusPolicy(Qt::StrongFocus);
        setFocus();
        setWindowTitle(_title.c_str());
    }

    std::map<std::string, std::string> InterfaceSelectorWidget::getInterfaces() const{
        
        std::map<std::string, std::string> result;

        for(auto &interface: interfaces_){
            result[interface->label()] =  interface->type();
        }

        return result;
    }


    void InterfaceSelectorWidget::updateInterfaces(int _nInterfaces){
        while(interfaces_.size() < _nInterfaces){
            interfaces_.push_back(new InterfaceElement);
            interfacesLayout_->addWidget(interfaces_.back());
        }
        
        while(interfaces_.size() > _nInterfaces){
            interfacesLayout_->removeWidget(interfaces_.back());
            delete interfaces_.back(); // also removes it from upper layout
            interfaces_.pop_back();
        }

        this->adjustSize();
        
    }
}