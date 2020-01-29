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

#include <mico/flow/blocks/visualizers/impl/DataVisualizer.h>
#include <QtWidgets>


namespace mico{
    class TreeWidgetItem : public QTreeWidgetItem {
    public:
    TreeWidgetItem(QTreeWidget* parent):QTreeWidgetItem(parent){}
    private:
    bool operator<(const QTreeWidgetItem &other)const {
        int column = treeWidget()->sortColumn();
        return text(column).toLower() < other.text(column).toLower();
    }
    };

    DataVisualizer::DataVisualizer(QWidget *parent){
        QVBoxLayout *mainLayout = new QVBoxLayout;
        mTree = new QTreeWidget();
        mTree->setColumnCount(8);
        mTree->setSortingEnabled(true);
        QStringList labels = {"Name", "id", "frames", "frame_aparitions", "cluster_aparitions", "clusters", "covisibility", "Point xyz"};
        mTree->setHeaderLabels(labels);
        mainLayout->addWidget(mTree);
        setLayout(mainLayout);
    }




    void DataVisualizer::updateData(std::map<int, std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>>> &_data){

        for(auto &df: _data){
            QTreeWidgetItem *dfTreeItem = new QTreeWidgetItem(mTree);

            // QTreeWidgetItem::setText(int column, const QString & text)
            dfTreeItem->setText(0, "DF");
            std::string dfid = std::to_string(df.second->id());
            dfTreeItem->setText(1, dfid.c_str());
            std::map<int,int> aparitions;
            for(auto &word:df.second->words()){
                QTreeWidgetItem *wordTreeItem = new QTreeWidgetItem();
                wordTreeItem->setText(0, "Word");
                std::string  wordId = std::to_string(word.second->id);
                wordTreeItem->setText(1, wordId.c_str());
                std::string frames;
                for(auto &frame: word.second->dfMap){
                    frames += std::to_string(frame.first) +", ";
                }
                wordTreeItem->setText(2, frames.c_str());
                std::string nFrames = std::to_string(word.second->dfMap.size());
                wordTreeItem->setText(3, nFrames.c_str());
                
                dfTreeItem->addChild(wordTreeItem);

            }
            std::string apaString = "";
            for(auto &counter:aparitions){
                apaString += std::to_string(counter.first) +"("+std::to_string(counter.second)+"),";
            }
            dfTreeItem->setText(4, apaString.c_str());

        }
    }

    void DataVisualizer::updateData(std::shared_ptr<mico::Dataframe<pcl::PointXYZRGBNormal>> _data){
        QTreeWidgetItem *dfTreeItem = new QTreeWidgetItem(mTree);

        // QTreeWidgetItem::setText(int column, const QString & text)
        dfTreeItem->setText(0, "DF");
        std::string dfid = std::to_string(_data->id());
        dfTreeItem->setText(1, dfid.c_str());
        std::map<int,int> aparitions;
        for(auto &word:_data->words()){
            QTreeWidgetItem *wordTreeItem = new QTreeWidgetItem();
            wordTreeItem->setText(0, "Word");
            std::string  wordId = std::to_string(word.first);
            wordTreeItem->setText(1, wordId.c_str());
            std::string frames;
            for(auto &frame: word.second->dfMap){
                frames += std::to_string(frame.first) +", ";
            }
            wordTreeItem->setText(2, frames.c_str());
            std::string nFrames = std::to_string(word.second->dfMap.size());
            wordTreeItem->setText(3, nFrames.c_str());
            
            dfTreeItem->addChild(wordTreeItem);

        }
        std::string apaString = "";
        for(auto &counter:aparitions){
            apaString += std::to_string(counter.first) +"("+std::to_string(counter.second)+"),";
        }
        dfTreeItem->setText(4, apaString.c_str());

    }


    void DataVisualizer::updateData(std::map<int, std::shared_ptr<mico::Word<pcl::PointXYZRGBNormal>>> &_data){
        QTreeWidgetItem *dfTreeItem = new QTreeWidgetItem(mTree);

        // QTreeWidgetItem::setText(int column, const QString & text)
        dfTreeItem->setText(0, "Words GBA");

        std::map<int,int> aparitions;
        for(auto &word:_data){
            QTreeWidgetItem *wordTreeItem = new QTreeWidgetItem();
            wordTreeItem->setText(0, "Word");
            std::string  wordId = std::to_string(word.second->id);
            wordTreeItem->setText(1, wordId.c_str());
            std::string frames;
            for(auto &frame: word.second->dfMap){
                frames += std::to_string(frame.first) +", ";
            }
            wordTreeItem->setText(2, frames.c_str());
            std::string nFrames = std::to_string(word.second->dfMap.size());
            wordTreeItem->setText(3, nFrames.c_str());
            
            dfTreeItem->addChild(wordTreeItem);

        }
        std::string apaString = "";
        for(auto &counter:aparitions){
            apaString += std::to_string(counter.first) +"("+std::to_string(counter.second)+"),";
        }
        dfTreeItem->setText(4, apaString.c_str());

        connect(mTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this,    SLOT(wordClicked(QTreeWidgetItem*, int)));

    }


    void DataVisualizer::wordClicked(QTreeWidgetItem *item, int column){

        auto idText = item->text(1);

        
        int wordId = idText.toInt();
        std::cout << "Clicked word: "+std::to_string(idText.toInt()) << std::endl;
        int aparitions = 0;

        int maxPerRow = sqrt(aparitions) + 1;
        int rows = aparitions/maxPerRow + ((aparitions%maxPerRow)== 0 ? 0:1);
        std::vector<std::vector<cv::Mat> > subplots(rows);
        int counter = 0;
        int currentRow = 0;
        
        if(counter < maxPerRow){
            for(;counter < maxPerRow; counter++){
                cv::Mat blackImg(480,640, CV_8UC3, cv::Scalar(0,0,0));
                subplots[currentRow].push_back(blackImg);
            }
        }

        cv::Mat display;
        for(unsigned i = 0; i <rows; i++){
            cv::Mat rowMat = subplots[i][0];
            for(unsigned j = 1 ; j < maxPerRow ;j++){
                cv::hconcat(rowMat, subplots[i][j], rowMat);
            }
            if(i == 0){
                display = rowMat;
            }else{
                cv::vconcat(display, rowMat, display);
            }
        }
        cv::namedWindow("words", CV_WINDOW_FREERATIO);
        cv::imshow("words", display);

        
    }
}