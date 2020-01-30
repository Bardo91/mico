//---------------------------------------------------------------------------------------------------------------------
//  MICO
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

#include <mico/flow/blocks/visualizers/impl/DataframeVisualizer.h>
#include <QtWidgets>


namespace mico{

    class ImageTab : public QWidget{
        public:
            ImageTab(cv::Mat _image, QWidget *parent = nullptr){
                cv::cvtColor(_image,_image,CV_BGR2RGB); //Qt reads in RGB whereas CV in BGR
                QImage imdisplay((uchar*)_image.data, _image.cols, _image.rows, _image.step, QImage::Format_RGB888); 
                imageDisplay_ = new QLabel;
                imageDisplay_->setPixmap(QPixmap::fromImage(imdisplay));
                QHBoxLayout *layout = new QHBoxLayout();
                layout->addWidget(imageDisplay_);
                setLayout(layout);
            }
        private:
            QLabel *imageDisplay_;
    };

    DataframeVisualizer::DataframeVisualizer(Dataframe<pcl::PointXYZRGBNormal>::Ptr _df, QWidget *parent){
        QVBoxLayout *mainLayout = new QVBoxLayout;
        setLayout(mainLayout);
        tabWidget_ = new QTabWidget;
        mainLayout->addWidget(tabWidget_);

        for(auto& otherDf: _df->covisibility()){
            if(otherDf->id() == _df->id())
                continue;

            cv::Mat imgOther = otherDf->leftImage();
            cv::Mat imgThis = _df->leftImage();
            
            int thisId = _df->id();
            int otherId = otherDf->id();
            for(auto &[wid, word]:_df->words()){
                if(word->isInFrame(otherId)){
                    cv::circle(imgOther, word->cvProjectionf(otherId), 3, cv::Scalar(0,255,0));
                    cv::putText(imgOther, std::to_string(word->id) , word->cvProjectionf(otherId), 2, 0.5, cv::Scalar(0,255,0), 0.5);

                    cv::circle(imgThis, word->cvProjectionf(thisId), 3, cv::Scalar(0,255,0));
                    cv::putText(imgThis, std::to_string(word->id), word->cvProjectionf(thisId), 2, 0.5, cv::Scalar(0,255,0), 0.5);
                }
            }
            cv::Mat finalImage;
            cv::hconcat(imgThis, imgOther, finalImage);

            std::stringstream label; label << _df->id() << "--" << otherDf->id();
            tabWidget_->addTab(new ImageTab(finalImage), label.str().c_str());
        }
    }



}