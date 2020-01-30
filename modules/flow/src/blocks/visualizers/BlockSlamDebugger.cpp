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

#include <mico/flow/blocks/visualizers/BlockSlamDebugger.h>

#include <QDialog>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>

#include <mico/base/map3d/Dataframe.h>
#include <mico/flow/blocks/visualizers/impl/DataVisualizer.h>

namespace mico{
    BlockSlamDebugger::BlockSlamDebugger(){
        createPolicy({{"Dataframe", "dataframe"}});
        

        registerCallback({ "Dataframe" }, 
                            [&](flow::DataFlow  _data){
                                lastDataframe_ = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Dataframe");
                                dataframesMap_[lastDataframe_->id()] = lastDataframe_;
                            }
                        );


    }
    
    BlockSlamDebugger::~BlockSlamDebugger(){
    
    }


    QWidget * BlockSlamDebugger::customWidget() {
        QGroupBox * box = new QGroupBox;
        
        QHBoxLayout * layout = new QHBoxLayout;
        QPushButton *visLastDf_ = new QPushButton("Visualize last Df");
        QPushButton *visAllDf_ = new QPushButton("Visualize all Dfs");
        layout->addWidget(visLastDf_);
        layout->addWidget(visAllDf_);
        
        box->setLayout(layout);

        QWidget::connect(visLastDf_, &QPushButton::clicked, [this](){
            DataVisualizer dv;
            dv.updateData(lastDataframe_);
            dv.show();
            dv.exec();
        });
        QWidget::connect(visAllDf_, &QPushButton::clicked, [this](){
            DataVisualizer dv;
            dv.updateData(dataframesMap_);
            dv.show();
            dv.exec();
        });

        return box;
    }

}

