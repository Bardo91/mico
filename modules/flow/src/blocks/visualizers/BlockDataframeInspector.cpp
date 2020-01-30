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

#include <mico/flow/blocks/visualizers/BlockDataframeInspector.h>

#include <QDialog>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>

#include <mico/base/map3d/Dataframe.h>
#include <mico/flow/blocks/visualizers/impl/DataframeVisualizer.h>

namespace mico{
    BlockDataframeInspector::BlockDataframeInspector(){
        createPolicy({{"Dataframe", "dataframe"}});
        

        registerCallback({ "Dataframe" }, 
                            [&](flow::DataFlow  _data){
                                auto df = _data.get<Dataframe<pcl::PointXYZRGBNormal>::Ptr>("Dataframe");
                                dataframes_[df->id()] = df;

                                QTreeWidgetItem *dfTreeItem = new QTreeWidgetItem(dfList_);
                                dfTreeItem->setText(0, std::to_string(df->id()).c_str());
                            }
                        );


    }
    
    BlockDataframeInspector::~BlockDataframeInspector(){
    
    }


    QWidget * BlockDataframeInspector::customWidget() {
        dfList_ = new QTreeWidget();

        
        QWidget::connect(dfList_, &QTreeWidget::itemClicked, [this](QTreeWidgetItem* _item, int _id){
            auto id = _item->text(0).toInt();
            DataframeVisualizer dv(this->dataframes_[id]);
            dv.show();
            dv.exec();
        });

        return dfList_;
    }

}

