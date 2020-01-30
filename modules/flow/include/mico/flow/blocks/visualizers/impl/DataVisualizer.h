//---------------------------------------------------------------------------------------------------------------------
//  mico_TOOLS
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

#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_IMPL_DATAVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_IMPL_DATAVISUALIZER_H_

#include <QDialog>
#include <QTreeWidget>

#include <mico/base/map3d/Dataframe.h>
#include <pcl/point_types.h>

#include <mico/base/map3d/Word.h>


namespace mico{
    class DataVisualizer : public QDialog {
        // Q_OBJECT

        public:
        explicit DataVisualizer(QWidget *parent = 0);


            void updateData(std::map<int, std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>> &_data);
            void updateData(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> _data);
            void updateData(std::map<int, std::shared_ptr<mico::Word<pcl::PointXYZRGBNormal>>> &_data);

        std::unordered_map<int, std::shared_ptr<mico::Word<pcl::PointXYZRGBNormal>>> mWordDictionary;

        // public slots:
            void wordClicked(QTreeWidgetItem *item, int column);
        protected:
            QTreeWidget *mTree;
            public:
    };
}



#endif // DATAVISUALIZER_H_