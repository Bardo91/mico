//---------------------------------------------------------------------------------------------------------------------
//  CORNER DETECTOR https://github.com/uzh-rpg/rpg_corner_events
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018
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

#include "mico/dvs/corner_detectors/harris_detector.h"

namespace mico{

  HarrisDetector::HarrisDetector(bool connect)
  : Detector(connect)
  {
    detectorName_ = "Harris";

    // parameters
    queueSize_ = 25;
    windowSize_ = 4;
    kernelSize_ = 5;
    harrisThreshold_ = 8.0;

    queues_ = new DistinctQueue(windowSize_, queueSize_, true);

    Eigen::VectorXd Dx = Eigen::VectorXd(kernelSize_);
    Eigen::VectorXd Sx = Eigen::VectorXd(kernelSize_);
    for (int i=0; i<kernelSize_; i++){
      Sx[i] = factorial(kernelSize_ - 1)/
              (factorial(kernelSize_ - 1 - i) * factorial(i));
      Dx[i] = pasc(i, kernelSize_-2) - pasc(i-1, kernelSize_-2);
    }
    Gx_ = Sx * Dx.transpose();
    Gx_ = Gx_ / Gx_.maxCoeff();

    const double sigma = 1.;
    const double A = 1./(2.*M_PI*sigma*sigma);
    const int l2 = (2*windowSize_+2-kernelSize_)/2;
    h_ = Eigen::MatrixXd(2*l2+1, 2*l2+1);
    for (int x=-l2; x<=l2; x++){
      for (int y=-l2; y<=l2; y++){
        const double h_xy = A * exp(-(x*x+y*y)/(2*sigma*sigma));
        h_(l2+x, l2+y) = h_xy;
      }
    }

    h_ /= h_.sum();
  }

  HarrisDetector::~HarrisDetector(){
  }

  bool HarrisDetector::isFeature(const dvs_msgs::Event &e){
    // update queues
    queues_->newEvent(e.x, e.y, e.polarity);

    // check if queue is full
    double score = harrisThreshold_ - 10.;
    if (queues_->isFull(e.x, e.y, e.polarity))
    {
      // check if current event is a feature
      score = getHarrisScore(e.x, e.y, e.polarity);

      lastScore_ = score;
    }

    return (score > harrisThreshold_);
  }

  double HarrisDetector::getHarrisScore(int img_x, int img_y, bool polarity){
    // do not consider border
    if (img_x<windowSize_ || img_x>sensorWidth_-windowSize_ ||
        img_y<windowSize_ || img_y>sensorHeight_-windowSize_){
        // something below the threshold
        return harrisThreshold_ - 10.;
    }

    const Eigen::MatrixXi local_frame = queues_->getPatch(img_x, img_y, polarity);

    const int l = 2*windowSize_+2-kernelSize_;
    Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(l, l);
    Eigen::MatrixXd dy = Eigen::MatrixXd::Zero(l, l);
  //  Eigen::MatrixXd dxy = Eigen::MatrixXd::Zero(l, l);
    for (int x=0; x<l; x++){
      for (int y=0; y<l; y++){
        for (int kx=0; kx<kernelSize_; kx++){
          for (int ky=0; ky<kernelSize_; ky++){
            dx(x, y) += local_frame(x+kx, y+ky)*Gx_(kx, ky);
            dy(x, y) += local_frame(x+kx, y+ky)*Gx_(ky, kx);
          }
        }
      }
    }

    double a=0., b=0., d=0.;
    for (int x=0; x<l; x++){
      for (int y=0; y<l; y++){
        a += h_(x, y) * dx(x, y) * dx(x, y);
        b += h_(x, y) * dx(x, y) * dy(x, y);
        d += h_(x, y) * dy(x, y) * dy(x, y);
      }
    }

    const double score = a*d-b*b - 0.04*(a+d)*(a+d);

    return score;
  }


  int HarrisDetector::factorial(int n) const{
    if (n > 1){
      return n * factorial(n - 1);
    }
    else{
      return 1;
    }
  }

  int HarrisDetector::pasc(int k, int n) const{
    if (k>=0 && k<=n){
      return factorial(n)/(factorial(n-k)*factorial(k));
    }
    else{
      return 0;
    }
  }

} // namespace
