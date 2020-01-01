// Resource from https://www.jianshu.com/p/be16847b0b74

#ifndef MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_
#define MICO_FLOW_MISC_PYTHON_CONVERSIONUTILS_H_

#include<opencv2/opencv.hpp>
#include<pybind11/pybind11.h>
#include<pybind11/numpy.h>

cv::Mat numpy_uint8_1c_to_cv_mat(pybind11::array_t<unsigned char>& input);

cv::Mat numpy_uint8_3c_to_cv_mat(pybind11::array_t<unsigned char>& input);

pybind11::array_t<unsigned char> cv_mat_uint8_1c_to_numpy(cv::Mat & input);

pybind11::array_t<unsigned char> cv_mat_uint8_3c_to_numpy(cv::Mat & input);

#endif 