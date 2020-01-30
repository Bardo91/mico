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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKUTILS2D_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKUTILS2D_H_

#include <flow/Block.h>

#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QLineEdit>

#include <opencv2/opencv.hpp>

#include <mutex>

namespace mico{

    struct Filter2D{
        virtual std::string name()                          = 0;
        virtual cv::Mat operator()(cv::Mat &_input)         = 0;
        virtual QWidget * customWidget()                    = 0;
    };

    class BlocksFilters2D: public flow::Block {
    public:
        virtual std::string name() const override {return "Filters 2D";}

        BlocksFilters2D();

        virtual QWidget * customWidget(){
            return visualContainer_;
        }

        std::string description() const override {return    "Block with various simple 2D filters for images.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};
    private:
        void initVisualization();

        std::vector<Filter2D *> filterList();

        void changeFilter(int _index);

    private:
        std::mutex filterGuard_;
        std::vector<Filter2D *> filterList_;
        Filter2D* currentFilter_;
        
        QComboBox *filterSelector_ = nullptr;
        QGroupBox *visualContainer_ = nullptr;
        QVBoxLayout *mainLayout_ = nullptr;

    };


    class BlocksImageConversion: public flow::Block {
    public:
        virtual std::string name() const override {return "Image Conversion";}

        BlocksImageConversion();

        virtual QWidget * customWidget(){
            return visualContainer_;
        }

        std::string description() const override {return    "Block image type conversion utilities.\n"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};

    private:
        void initVisualization();

        void changeFilter(int _index);

    private:
        std::mutex guard_;
        std::vector<std::pair<std::string, int>> typeConversion_;
        std::pair<std::string, int> currentType_;
        
        QComboBox *conversionSelector_ = nullptr;
        QGroupBox *visualContainer_ = nullptr;
        QVBoxLayout *mainLayout_ = nullptr;

    };


    //-----------------------------------------------------------------------------------------------------------------
    //-------------------------------------------- IMPLEMENTATIONS ----------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------
    struct Filter2DSobel: public Filter2D{
        virtual std::string name()                      override {return "Sobel";}
        virtual cv::Mat operator()(cv::Mat &_input)     override ;
        virtual QWidget * customWidget()                override {return nullptr;}
    };

    struct Filter2DLaplacian: public Filter2D{
        virtual std::string name()                      override {return "Laplacian";}
        virtual cv::Mat operator()(cv::Mat &_input)     override;
        virtual QWidget * customWidget()                override {return nullptr;}
    };

    struct Filter2DGaussian: public Filter2D{
        Filter2DGaussian();
        virtual std::string name()                      override {return "Gaussian";}
        virtual cv::Mat operator()(cv::Mat &_input)     override;
        virtual QWidget * customWidget()                override {return kernelSizeEdit_;}
    private:
        QGroupBox *configWidget_;
        QLineEdit *kernelSizeEdit_;
    };

}

#endif