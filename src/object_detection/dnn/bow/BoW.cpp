//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#include <fstream>
#include <numeric>

#include <rgbd_tools/object_detection/dnn/bow/BoW.h>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace rgbd {
	// FAST WRAPER
	class FASTwrapper: public FeatureDetector {
	public:
		static Ptr<FASTwrapper> create() { return Ptr<FASTwrapper>();}
		void detect(InputArray image, std::vector<KeyPoint>& keypoints, InputArray mask) {
			FAST(image, keypoints, 9);
		}
	};


	//-----------------------------------------------------------------------------------------------------------------
	void BoW::params(Params _params) {
		mParams = _params;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::train(std::string _imagePathTemplate, std::string _gtFile) {
		Ptr<ml::TrainData> trainData =  createTrainData(_imagePathTemplate, _gtFile);
		train(trainData);
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<std::pair<unsigned, float>> BoW::evaluate(Mat _image, vector<Rect> _regions) {
		std::vector<std::pair<unsigned, float>> topics;
		std::vector<Mat> descriptors;
		std::vector<Mat> histograms;
		
		vector<KeyPoint> keypoints;
		Mat imageDescriptors = computeFeatures(_image, keypoints);

		for (Rect region : _regions) {
			Mat regionDescriptors;
			for (unsigned i = 0; i < keypoints.size(); i++) {
				if (region.contains(keypoints[i].pt)) {
					regionDescriptors.push_back(imageDescriptors.row(i));
				}
			}
			descriptors.push_back(regionDescriptors);
		}

		if(descriptors.size() == 0){
			std::cout << "Error, getting empty descriptors" << std::endl;
			return topics;
		}

		vectorQuantization(descriptors, histograms);
		Mat newData;
		for (Mat histogram : histograms) {
			double max;
			minMaxLoc(histogram, NULL, &max);
			histogram = histogram / max;

			Mat rotated;
			transpose(histogram, rotated);
			newData.push_back(rotated);
		}
		float label, prob;
		predict(newData, label, prob);
		
		// 666 recoverall data!
		//for (unsigned i = 0; i < results.rows; i++) {
			topics.push_back(std::pair<unsigned, float>(label, prob));
		//}
		return topics;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::save(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::WRITE);
		fs << "vocabulary" << mCodebook;
		fs.release();
		mSvm->save(_name + ".svm");
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::load(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::READ);
		fs["vocabulary"] >> mCodebook;
		fs.release(); 
		mSvm =  Algorithm::load<SVM>(_name + ".svm");
	}

	//-----------------------------------------------------------------------------------------------------------------
	cv::Mat BoW::loadImage(std::string _filePatern, unsigned _index){
		unsigned imageId = _index;
		string fileName = _filePatern.substr(0, _filePatern.find("%d")) + to_string(_index) + _filePatern.substr(_filePatern.find("%d")+2, _filePatern.length());
		Mat img = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);	//	Load image using file path and as grayscale image.

		if(img.rows == 0)
			return img;

		double factor = 300.0/(img.rows > img.cols ? img.rows: img.cols);
		resize(img, img, Size(), factor, factor);
		return img;

	}

	//-----------------------------------------------------------------------------------------------------------------
	Ptr<ml::TrainData> BoW::createTrainData(const std::string &_imagePathTemplate, const std::string &_gtFile) {
		Mat descriptorsAll;
		vector<Mat> descriptorPerImg;
		unsigned index = 1;
		for (;;) {	// For each image in dataset
			Mat frame = loadImage(_imagePathTemplate, index);
			if (frame.rows == 0) {
				break;	// Can't open image.
			}
			double scaleFactor = 1;
			for (unsigned i = 0; i < mParams.nScalesTrain; i++) {
				Mat resizedImage;
				resize(frame, resizedImage, Size(), scaleFactor, scaleFactor);

				// Look for interest points to compute features in there.
				vector<KeyPoint> keypoints;
				Mat descriptors = computeFeatures(resizedImage, keypoints);

				Mat descriptors_32f;
				descriptors.convertTo(descriptors_32f, CV_32F, 1.0 / 255.0);

				descriptorsAll.push_back(descriptors_32f);
				descriptorPerImg.push_back(descriptors_32f);
				scaleFactor*=mParams.scaleFactor;
			}
			index++;
		}
		// Form codebook from all descriptors
		mCodebook = formCodebook(descriptorsAll);
		descriptorsAll.release();

		// Compute histograms
		std::vector<cv::Mat> histograms;
		vectorQuantization(descriptorPerImg, histograms);

		// TRAIN DATA
		Mat X(histograms.size(), mCodebook.rows, CV_32FC1);
		for (int i = 0; i < histograms.size(); i++) {
			Mat rotated;
			transpose(histograms[i], rotated);
			rotated.copyTo(X.row(i));
		}

		Mat groundTruth = loadGroundTruth(_gtFile);

		std::cout << groundTruth.rows << ", " << groundTruth.cols << std::endl;
		std::cout << X.rows << ", " << std::endl;

		return ml::TrainData::create(X, ml::SampleTypes::ROW_SAMPLE, groundTruth);
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::computeFeatures(const Mat &_frame, vector<KeyPoint> &_keypoints) {
		Ptr<FeatureDetector> detector;
		Ptr<FeatureDetector> descriptor;

		switch (mParams.extractorType) {
		case Params::eExtractorType::SIFT:
			detector = xfeatures2d::SIFT::create();
			break;
		case Params::eExtractorType::SURF:
			detector = xfeatures2d::SURF::create();
			break;
		case Params::eExtractorType::FAST:
			detector = FASTwrapper::create();
			break;
		case Params::eExtractorType::ORB:
			detector = ORB::create();
			break;
		}

		switch (mParams.descriptorType) {
		case Params::eDescriptorType::SIFT:
			descriptor = xfeatures2d::SIFT::create();
			break;
		case Params::eDescriptorType::SURF:
			descriptor = xfeatures2d::SURF::create();
			break;
		case Params::eDescriptorType::ORB:
			descriptor = ORB::create();
			break;
		}

		Mat descriptors;
		detector->detect(_frame, _keypoints);
		descriptor->compute(_frame, _keypoints, descriptors);

		return descriptors;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::formCodebook(const Mat &_descriptors) {
		Mat codebook;
		int codebookSize = mParams.vocSize;
		TermCriteria criteria(CV_TERMCRIT_ITER,100,0.001);
		vector<int> labels;
		std::cout << "Number of descriptors " << _descriptors.rows << std::endl;
		std::cout << "Performing kmeans" << std::endl;
		kmeans(_descriptors, codebookSize, labels, criteria, 1,KMEANS_PP_CENTERS, codebook);
		//std::cout << "Codebook size " << codebook.rows << std::endl;
		return codebook;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::vectorQuantization(const std::vector<cv::Mat> &_descriptors, std::vector<cv::Mat> &_histograms) {
		std::cout << "Performing vector quantization" << std::endl;
		for (Mat mat : _descriptors) {
			_histograms.push_back(Mat(mCodebook.rows, 1 , CV_32F, Scalar(0)));
			for (int i = 0; i < mat.rows; i++) {
				double minDist = 999999;
				unsigned index = 0;

				// Compute dist of descriptor i to centroid j;
				for (int j = 0; j < mCodebook.rows; j++) {
					double dist = 0;
					for (int k = 0; k < mCodebook.cols; k++) {
						double a = mCodebook.at<float>(j,k) - mat.at<float>(i,k);
						dist += a*a;
					}
					//dist = sqrt(dist);
					if (dist < minDist) {
						minDist = dist;
						index = j;
					}
				}
				_histograms[_histograms.size()-1].at<float>(index) += 1;
			}
			int recount = (int) sum(_histograms[_histograms.size() -1])[0];
			assert(recount == mat.rows);
		}

		// Normalize histograms
		for (cv::Mat &histogram : _histograms) {
			double max;
			minMaxLoc(histogram, NULL, &max);
			histogram = histogram / max;
		}

		FileStorage histogramsFile("histogramPerImg.yml", FileStorage::WRITE);
		for (unsigned i = 0; i < _histograms.size(); i++) {
			histogramsFile << "hist_"+to_string(i) << _histograms[i];
		}

		std::cout << "End vector quantization" << std::endl;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::loadGroundTruth(const std::string & _gtFile) const{
		Mat groundTruth(0, 1, CV_32SC1);
		ifstream gtFile(_gtFile);
		assert(gtFile.is_open());
		for (unsigned i = 0; !gtFile.eof();i++) {
			int gtVal;
			gtFile >> gtVal;
			for (unsigned j = 0; j < mParams.nScalesTrain; j++) {
				groundTruth.push_back(gtVal);
			}

		}
		return groundTruth;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::train(const Ptr<ml::TrainData> &_trainData) {
		mSvm = SVM::create();
		mSvm->setType(mParams.svmType);
		mSvm->setKernel(mParams.svmKernel);
		mSvm->setGamma(mParams.gamma);
		mSvm->setC(mParams.c);
		mSvm->train(_trainData);
		//mSvm->trainAuto(trainData, 10, ParamGrid(1,1000,1.5), ParamGrid(0.00001,1,2));
		std::cout << "C: " << mSvm->getC() << ". Gamma: " << mSvm->getGamma() << std::endl;
		std::cout << "finished training" << std::endl;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::predict(const cv::Mat & _newData, float& _label, float &_prob) {
		_label = mSvm->predict(_newData);
		_prob = 1;
	}

	//-----------------------------------------------------------------------------------------------------------------

}	// namespace algorithm