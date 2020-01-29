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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKIMAGEVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKIMAGEVISUALIZER_H_

#include <flow/Block.h>

#include <vtkJPEGReader.h>
#include <vtkImageData.h>
#include <vtkImageMapper.h> // Note: this is a 2D mapper (cf. vtkImageActor which is 3D)
#include <vtkActor2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <opencv2/opencv.hpp>

namespace mico{

    class BlockImageVisualizer: public flow::Block{
    public:
        virtual std::string name() const override {return "Image Visualizer";}

        BlockImageVisualizer();
        // ~BlockImageVisualizer(){};

        std::string description() const override {return    "Simple image visualizer block. Compatible with RGB and Depth images.\n"
                                                            "   - Inputs: \n";};

    private:
        vtkSmartPointer<vtkImageData> convertCVMatToVtkImageData(const cv::Mat &sourceCVImage, bool flipOverXAxis);
        vtkSmartPointer<vtkImageData> convertCVMatToVtkImageDataDepth(const cv::Mat &sourceCVImage, bool flipOverXAxis);

    private:
        vtkSmartPointer<vtkImageMapper> mapper_;
        vtkSmartPointer<vtkActor2D> image_;
        vtkSmartPointer<vtkRenderer> renderer_;
        vtkSmartPointer<vtkRenderWindow> window_;

        bool idle_ = true;
    };

}

#endif