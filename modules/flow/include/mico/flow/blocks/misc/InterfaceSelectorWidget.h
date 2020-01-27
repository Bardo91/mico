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
        InterfaceElement(std::string _label = "label");

        std::string label() const{
            return label_->text().toStdString();
        }
        std::string type() const{
            return typeList_->currentText().toStdString();
        }

    public:
        QComboBox *typeList_;
        QLineEdit *label_;
    };

    class InterfaceSelectorWidget : public QDialog{
    public:
        enum class INTERFACE_TYPE {INPUT, OUTPUT};
        InterfaceSelectorWidget(std::string _title, bool _hasInputs = true, bool _hasOutputs = true, QWidget *parent = nullptr);

        std::map<std::string, std::string> getInterfaces(INTERFACE_TYPE _type) const;

        // Ugly method but I cannot find another way by now.
        void callThisIfSmthChangeInside(std::function<void(void)> _fn){
            callbackIfChangeInside_ = _fn;
        }

    private:
        void updateInterfacesIn(int _nInterfaces);
        void updateInterfacesOut(int _nInterfaces);

    private:
        QSpinBox *countSelectorIn_, *countSelectorOut_;
        QHBoxLayout *twoColumnLayout_;
        QVBoxLayout *interfacesLayoutIn_, *interfacesLayoutOut_;
        QGroupBox *inputGroup_, *outputGroup_;
        std::vector<InterfaceElement*> interfacesInput_;
        std::vector<InterfaceElement*> interfacesOutput_;

        bool hasInputs_ = false, hasOutputs_ = false;

        std::function<void(void)>callbackIfChangeInside_;
    };



}

#endif