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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_MISC_INTERFACESELECTORWIDGET_H_
#define MICO_FLOW_STREAMERS_BLOCKS_MISC_INTERFACESELECTORWIDGET_H_

#include <QDialog>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>

namespace mico{
    class InterfaceElement: public QGroupBox{
    public:
        InterfaceElement();

        std::string label() const{
            return label_->text().toStdString();
        }
        std::string type() const{
            return typeList_->currentText().toStdString();
        }

    private:
        QComboBox *typeList_;
        QLineEdit *label_;
    };

    class InterfaceSelectorWidget : public QDialog{
    public:
        InterfaceSelectorWidget(std::string _title, QWidget *parent = nullptr);

        std::map<std::string, std::string> getInterfaces() const;

    private:
        void updateInterfaces(int _nInterfaces);

    private:
        QSpinBox *countSelector_;
        QVBoxLayout *interfacesLayout_;
        QVBoxLayout *mainLayout_;
        std::vector<InterfaceElement*> interfaces_;

    };



}

#endif