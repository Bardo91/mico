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


#include <mico/flow/blocks/misc/BlocksUtils2D.h>

#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>

#include <QLabel>


namespace mico{
    BlocksFilters2D::BlocksFilters2D(){
        filterList_ = {     new Filter2DSobel,
                            new Filter2DLaplacian,
                            new Filter2DGaussian    };

        initVisualization();

        createPipe("Filtered", "image");

        createPolicy({     {"Input","image"} });
        
        registerCallback({"Input"}, 
                                [&](flow::DataFlow _data){
                                    if(getPipe("Filtered")->registrations() !=0 ){
                                        auto image = _data.get<cv::Mat>("Input"); 
                                        filterGuard_.lock();
                                        auto output = (*currentFilter_)(image);
                                        filterGuard_.unlock();
                                        getPipe("Filtered")->flush(output);
                                    }
                                });
    }

    void BlocksFilters2D::initVisualization(){
        visualContainer_ = new QGroupBox();
        mainLayout_ = new QVBoxLayout();
        visualContainer_->setLayout(mainLayout_);

        filterSelector_ = new QComboBox();
        mainLayout_->addWidget(filterSelector_);

        for(auto &filter:filterList()){
            filterSelector_->addItem(filter->name().c_str());
        }

        QWidget::connect(filterSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int _n){ this->changeFilter(_n); });
        currentFilter_ = filterList_[0];
        filterSelector_->setCurrentIndex(0);
    }

    std::vector<Filter2D*> BlocksFilters2D::filterList(){
        return filterList_;
    }


    void BlocksFilters2D::changeFilter(int _index){
        filterGuard_.lock();

        if(currentFilter_->customWidget()){
            currentFilter_->customWidget()->setVisible(false);
            mainLayout_->removeWidget(currentFilter_->customWidget());
        }
        currentFilter_ = filterList_[_index];
        auto widget = currentFilter_->customWidget();
        if(widget){
            widget->setVisible(true);
            mainLayout_->addWidget(widget);
        }

        // filterSelector_->adjustSize();
        filterGuard_.unlock();
    }

    //-----------------------------------------------------------------------------------------------------------------
    BlocksImageConversion::BlocksImageConversion(){
        typeConversion_ = {     {"RGB2GRAY", cv::COLOR_BGR2GRAY},
                                {"GRAY2BGR", cv::COLOR_GRAY2BGR},
                                {"RGB2HSV", cv::COLOR_RGB2HSV},
                                {"HSV2RGB", cv::COLOR_HSV2BGR}    };

        initVisualization();

        createPipe("Output", "image");

        createPolicy({     {"Input","image"} });
        
        registerCallback({"Input"}, 
                                [&](flow::DataFlow _data){
                                    if(getPipe("Output")->registrations() !=0 ){
                                        auto image = _data.get<cv::Mat>("Input"); 
                                        cv::Mat output;
                                        guard_.lock();
                                        cv::cvtColor(image, output, currentType_.second);
                                        guard_.unlock();
                                        getPipe("Output")->flush(output);
                                    }
                                });
    }

    void BlocksImageConversion::initVisualization(){
        visualContainer_ = new QGroupBox();
        mainLayout_ = new QVBoxLayout();
        visualContainer_->setLayout(mainLayout_);

        conversionSelector_ = new QComboBox();
        mainLayout_->addWidget(conversionSelector_);

        for(auto &conversion:typeConversion_){
            conversionSelector_->addItem(conversion.first.c_str());
        }

        QWidget::connect(conversionSelector_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int _n){ this->changeFilter(_n); });
        currentType_ = typeConversion_[0];
        conversionSelector_->setCurrentIndex(0);
    }


    void BlocksImageConversion::changeFilter(int _index){
        guard_.lock();

        currentType_ = typeConversion_[_index];

        // conversionSelector_->adjustSize();
        guard_.unlock();
    }



    //-----------------------------------------------------------------------------------------------------------------
    //-------------------------------------------- IMPLEMENTATIONS ----------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------

    cv::Mat Filter2DSobel::operator()(cv::Mat &_input){
        cv::Mat src_gray;
        int ddepth = CV_16S;
        if(_input.channels() == 3)
            cv::cvtColor(_input, src_gray, cv::COLOR_BGR2GRAY);
        else
            src_gray = _input.clone();
        
        cv::Mat grad, grad_x, grad_y, abs_grad_x, abs_grad_y;
        cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3);
        cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3);

        // converting back to CV_8U
        cv::convertScaleAbs(grad_x, abs_grad_x);
        cv::convertScaleAbs(grad_y, abs_grad_y);

        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        cv::Mat output;
        if(_input.channels() == 3)
            cv::cvtColor(grad, output, cv::COLOR_GRAY2BGR);

        return output;
    }

    cv::Mat Filter2DLaplacian::operator()(cv::Mat &_input){
        cv::Mat output;
        int ddepth = CV_16S;

        cv::Mat src_gray;
        if(_input.channels() == 3)
            cv::cvtColor( _input, src_gray, cv::COLOR_BGR2GRAY ); // Convert the image to grayscale
        else
            src_gray = _input.clone();
        
        
        cv::Mat dst, abs_dst;
        cv::Laplacian( src_gray, dst, ddepth );
        // converting back to CV_8U
        cv::convertScaleAbs( dst, abs_dst );

        if(_input.channels() == 3)
            cv::cvtColor( abs_dst, output, cv::COLOR_GRAY2BGR ); // Convert the image to grayscale

        return output;
    }

    cv::Mat Filter2DGaussian::operator()(cv::Mat &_input){
        std::stringstream ss;
        int kernelSize;
        ss << kernelSizeEdit_->text().toStdString();
        ss >> kernelSize;
        
        cv::Mat output;
        if(kernelSize % 2 != 0)
            GaussianBlur(_input, output, cv::Size(kernelSize, kernelSize), 0, 0, cv::BORDER_DEFAULT);
        
        return output;

    }

    Filter2DGaussian::Filter2DGaussian(){
        configWidget_ = new QGroupBox();
        auto mainLayout = new QHBoxLayout();
        configWidget_->setLayout(mainLayout);

        QLabel *label = new QLabel("Kernel Size: ");
        mainLayout->addWidget(label);
        kernelSizeEdit_ = new QLineEdit("3");
        mainLayout->addWidget(kernelSizeEdit_);
        
        QWidget::connect(kernelSizeEdit_, &QLineEdit::textChanged, [this](const QString &text) {
            std::stringstream ss;
            int kernelSize;
            ss << kernelSizeEdit_->text().toStdString();
            ss >> kernelSize;   
            
            if(kernelSize % 2 == 0)
                kernelSizeEdit_->setStyleSheet("border: 1px solid red");
            else
               kernelSizeEdit_->setStyleSheet("border: 1px solid green");
            
        });

    }

}
