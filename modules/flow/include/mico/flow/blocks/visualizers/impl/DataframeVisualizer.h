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

#ifndef MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_IMPL_DATAFRAMEVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_VISUALIZERS_IMPL_DATAFRAMEVISUALIZER_H_

#include <QDialog>
#include <QTreeWidget>

#include <mico/base/map3d/Dataframe.h>
#include <pcl/point_types.h>

#include <mico/base/map3d/Word.h>

#include <QTabWidget>

namespace mico{
    class DataframeVisualizer : public QDialog {
    public:
        explicit DataframeVisualizer(Dataframe<pcl::PointXYZRGBNormal>::Ptr , QWidget *parent = 0);

    private:
        QTabWidget *tabWidget_;

    };
}



#endif // DataframeVisualizer_H_