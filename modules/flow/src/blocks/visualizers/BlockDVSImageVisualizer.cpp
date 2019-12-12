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



#include <mico/flow/blocks/visualizers/BlockDVSImageVisualizer.h>

#include <flow/Policy.h>


namespace mico{

    BlockDVSImageVisualizer::BlockDVSImageVisualizer(){

        mapper_ = vtkSmartPointer<vtkImageMapper>::New();
        
        image_ = vtkSmartPointer<vtkActor2D>::New();
        image_->SetMapper(mapper_);

        renderer_ = vtkSmartPointer<vtkRenderer>::New();
        renderer_->AddActor(image_);

        window_ = vtkSmartPointer<vtkRenderWindow>::New();
        window_->AddRenderer(renderer_);

        createPolicy({{"DVS Events","v-event"}});

        registerCallback({"DVS Events"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;  
                                        
                                        dvs_msgs::EventArray events = _data.get<dvs_msgs::EventArray>("DVS Events");
                                        cv::Mat fakeImage = convertEventsToCVMat(events);
                                        cv::imshow("ff",fakeImage);
                                        cv::waitKey(0);
                                        
                                        auto vtkImage = convertCVMatToVtkImageData(fakeImage, false );
                                        mapper_->SetInputData(vtkImage);
                                        mapper_->SetColorWindow(255); // width of the color range to map to
                                        mapper_->SetColorLevel(127.5); // center of the color range to map to

                                        int imageSize[3];
                                        vtkImage->GetDimensions(imageSize);
                                        window_->SetSize(imageSize[0], imageSize[1]);

                                        window_->Render();
                                        idle_ = true;
                                    }

                                }
                            );
    }

    vtkSmartPointer<vtkImageData> BlockDVSImageVisualizer::convertCVMatToVtkImageData(const cv::Mat &_sourceCVImage, bool _flipOverXAxis) {
        vtkSmartPointer<vtkImageData> outputVtkImage = vtkSmartPointer<vtkImageData>::New();
        double spacing[3] = {1, 1, 1};
        double origin[3] = {0, 0, 0};
        int extent[6] = {0, _sourceCVImage.cols - 1, 0, _sourceCVImage.rows - 1, 0, 0};
        auto numOfChannels = _sourceCVImage.channels();
        outputVtkImage->SetSpacing(spacing);
        outputVtkImage->SetOrigin(origin);
        outputVtkImage->SetExtent(extent);
        // outputVtkImage->SetDimensions(_sourceCVImage.cols, _sourceCVImage.rows, 1);
        outputVtkImage->AllocateScalars(VTK_UNSIGNED_CHAR, numOfChannels);

        cv::Mat tempCVImage;
        if (_flipOverXAxis) { // Normaly you should flip the image!
            cv::flip(_sourceCVImage, tempCVImage, 0);
        }
        else {
            tempCVImage = _sourceCVImage;
        }
        
        unsigned char* dptr = reinterpret_cast<unsigned char*>(outputVtkImage->GetScalarPointer());
        mempcpy(dptr, tempCVImage.data, _sourceCVImage.cols*_sourceCVImage.rows*3);

        outputVtkImage->Modified();

        return outputVtkImage;
    }

    cv::Mat BlockDVSImageVisualizer::convertEventsToCVMat(const dvs_msgs::EventArray &_sourceEvents){
        cv::Mat image(_sourceEvents.width, _sourceEvents.height, CV_8UC3, cv::Scalar(0,0,0));
        
        for (auto event:_sourceEvents.events){
            image.at<cv::Vec3b>(cv::Point(event.x, event.y)) = (
                event.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
        }

        return image;
    }
}
