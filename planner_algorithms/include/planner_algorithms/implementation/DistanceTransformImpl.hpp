/*
 * DistanceTransformImpl.hpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_DISTANCETRANSFORMIMPL_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_DISTANCETRANSFORMIMPL_HPP_

#include <opencv2/imgproc/imgproc.hpp>
#include <planner_interfaces/Exceptions.hpp>
#include <planner_algorithms/OpenCvConversions.hpp>

namespace planning2d
{
namespace algorithms
{

template <typename T>
void distanceTransform(const Map<T>& in, Map<float>& out, Map<int32_t>* labels = nullptr, int distanceType /*= CV_DIST_L2*/, int maskSize /*= CV_DIST_MASK_PRECISE*/)
{
  SM_ASSERT_TRUE(planning2d::FunctionInputException, !in.empty(), "You must not call this method with an empty grid");
  SM_ASSERT_TRUE(planning2d::FunctionInputException, out.hasEqualMetric(in), "");
  SM_ASSERT_TRUE(planning2d::FunctionInputException, labels == nullptr || labels->hasEqualMetric(in), "");

  // create the opencv matrices using the memory of the Eigen matrices
  cv::Mat cvSrc = convertToCvMat<uint8_t>(in.matrix());
  cv::Mat cvDest = toCvMat(out.matrix());

  // convert to correct input data format
  cv::Mat cvSrc2;
  cvSrc.convertTo(cvSrc2, cv::DataType<uint8_t>::type);

  // run distance transform
  if (labels != nullptr)
  {
    cv::Mat cvLabels = toCvMat(*labels);
    cv::distanceTransform(cvSrc, cvDest, cvLabels, distanceType, maskSize, cv::DIST_LABEL_CCOMP);
  }
  else
  {
    cv::distanceTransform(cvSrc, cvDest, distanceType, maskSize);
  }

  // apply metric information since distanceTransform will compute the distances in pixels
  out.matrix() *= out.resolution();
}

template <typename T>
void signedDistanceTransform(const Map<T>& in, Map<float>& out, Map<int32_t>* labels /*= nullptr*/, int distanceType /*= CV_DIST_L2*/, int maskSize /*= CV_DIST_MASK_PRECISE*/)
{
  distanceTransform(in, out, labels, distanceType, maskSize);

  // Run the distance transform on an inverted version to compute the distances inside the obstacles
  Map<T> inv = in;
  inv.invert();
  Map<float> invDt(in.getOrigin(), in.resolution(), in.sizeInCells());
  distanceTransform(inv, invDt, nullptr, distanceType, maskSize);

  // invert the distances of the inverted grid and add the distance of one cell,
  // so that the border of the obstacles gets the distance 0.0 in the end
  invDt.matrix() = -invDt.matrix().array() + inv.resolution();

  // combine the results, overwrite the values inside the obstacles (distance 0.0)
  // with the inverted transform
  out.matrix() = (out.matrix().array() == 0.0).select(invDt.matrix(), out.matrix());
}


extern template void distanceTransform(const Map<OccupancyValue>& in, Map<float>& out, Map<int32_t>* labels, int distanceType, int maskSize);
extern template void distanceTransform(const Map<uint8_t>& in, Map<float>& out, Map<int32_t>* labels, int distanceType, int maskSize);

} /* namespace algorithms */
} /* planning2d */


#endif /* INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_DISTANCETRANSFORMIMPL_HPP_ */
