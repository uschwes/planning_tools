/*
 * OpenCvConversions.hpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_OPENCVCONVERSIONS_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_OPENCVCONVERSIONS_HPP_

// opencv2
#include <opencv2/core/core.hpp>

// Eigen
#include <Eigen/Dense>

// planner interfaces
#include <planner_interfaces/OccupancyGrid.hpp>

namespace cv
{
template<> class DataDepth<planning2d::OccupancyValue> { public: enum { value = CV_8U, fmt=(int)'u' }; };

template<> class DataType<planning2d::OccupancyValue>
{
public:
    typedef uint8_t value_type;
    typedef uint8_t work_type;
    typedef value_type channel_type;
    typedef value_type vec_type;
    enum { generic_type = 0, depth = DataDepth<channel_type>::value, channels = 1,
           fmt=DataDepth<channel_type>::fmt, type = CV_MAKETYPE(depth, channels) };
};
} /* namespace cv */

namespace planning2d
{

/// \brief Converts Eigen matrix to opencv matrix by mapping the underlying memory.
///        That means that modifying returned matrix will modify the input matrix
template <typename Matrix>
cv::Mat toCvMat(Matrix& mat)
{
  static_assert(Matrix::Options & Eigen::RowMajor, "Conversion to opencv currently not supported for column-major matrices");
  // create the opencv matrices using the memory of the Eigen matrices
  // cv::Mat assignment operator performs no deep copy, so we return by value
  return cv::Mat(mat.rows(), mat.cols(), cv::DataType<typename Matrix::Scalar>::type,(void*)mat.data(), mat.stride()*sizeof(typename Matrix::Scalar));
}

/// \brief Converts Eigen matrix to opencv matrix.
///        Underlying memory will be mapped but cannot be modified later, so the input matrix is
///        guaranteed to be const.
template <typename Matrix>
const cv::Mat toCvMat(const Matrix& mat)
{
  return toCvMat(const_cast<Matrix&>(mat));
}

/// \brief Converts planning2d::Map to opencv matrix
template <typename T>
cv::Mat toCvMat(Map<T>& map) { return toCvMat(map.matrix()); }

/// \brief Converts an Eigen matrix to an opencv matrix of specified type
///        If output type matches input type, no data will be copied. In that case, the input matrix
///        might be altered through the returned opencv matrix later despite the const specifier.
///        That's how opencv works...
template <typename T, typename Matrix>
const cv::Mat convertToCvMat(const Matrix& mat)
{
//  static_assert(cv::DataType<T>::type != cv::DataType<typename Derived::Scalar>::type, "");
  const cv::Mat cvMat = toCvMat(mat);
  cv::Mat cvMat2;
  if ((int)cv::DataType<T>::type != (int)cv::DataType<typename Matrix::Scalar>::type)
    cvMat.convertTo(cvMat2, cv::DataType<T>::type);
  else
    cvMat2 = cvMat;
  return cvMat2;
}

} /* namespace planning2d */


#endif /* INCLUDE_PLANNER_ALGORITHMS_OPENCVCONVERSIONS_HPP_ */
