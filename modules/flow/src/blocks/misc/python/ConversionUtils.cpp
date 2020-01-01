
// Resource from https://www.jianshu.com/p/be16847b0b74

#include <mico/flow/blocks/misc/python/ConversionUtils.h>
#include <pybind11/numpy.h>

/*
Python->C++ Mat
*/


cv::Mat numpy_uint8_1c_to_cv_mat(pybind11::array_t<unsigned char>& input) {

    if (input.ndim() != 2)
        throw std::runtime_error("1-channel image must be 2 dims ");

    pybind11::buffer_info buf = input.request();

    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC1, (unsigned char*)buf.ptr);
    
    return mat;
}


cv::Mat numpy_uint8_3c_to_cv_mat(pybind11::array_t<unsigned char>& input) {

    if (input.ndim() != 3)
        throw std::runtime_error("3-channel image must be 3 dims ");

    pybind11::buffer_info buf = input.request();

    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (unsigned char*)buf.ptr);

    return mat;
}


/*
C++ Mat ->numpy
*/
pybind11::array_t<unsigned char> cv_mat_uint8_1c_to_numpy(cv::Mat& input) {

    pybind11::array_t<unsigned char> dst = pybind11::array_t<unsigned char>({ input.rows,input.cols }, input.data);
    return dst;
}

pybind11::array_t<unsigned char> cv_mat_uint8_3c_to_numpy(cv::Mat& input) {

    pybind11::array_t<unsigned char> dst = pybind11::array_t<unsigned char>({ input.rows,input.cols,3}, input.data);
    return dst;
}


