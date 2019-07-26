/*
 * DistanceTransformPy.cpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <boost/python.hpp>

#include <planner_algorithms/DistanceTransform.hpp>

using namespace boost::python;
using namespace planning2d;

template <typename T>
Map<float> distanceTransformWrapper(const Map<T>& map, int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE)
{
  Map<float> dt(map.getOrigin(), map.resolution(), map.sizeInCells());
  algorithms::distanceTransform(map, dt, nullptr, distanceType, maskSize);
  return dt;
}

template <typename T>
boost::python::tuple distanceTransformLabeledWrapper(const Map<T>& map, int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE) {
  Map<float> dt(map.getOrigin(), map.resolution(), map.sizeInCells());
  Map<int32_t> labels(map.getOrigin(), map.resolution(), map.sizeInCells());
  algorithms::distanceTransform(map, dt, &labels, distanceType, maskSize);
  return boost::python::make_tuple(dt, labels);
}

template <typename T>
Map<float> signedDistanceTransformWrapper(const Map<T>& map, int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE) {
  Map<float> dt(map.getOrigin(), map.resolution(), map.sizeInCells());
  algorithms::signedDistanceTransform(map, dt, nullptr, distanceType, maskSize);
  return dt;
}

template <typename T>
void exportDistanceTransformT(const std::string& name)
{
  def("distanceTransform", &distanceTransformWrapper<T>, (boost::python::arg("distanceType") = (int)CV_DIST_L2, boost::python::arg("maskSize") = (int)CV_DIST_MASK_PRECISE),
      ("MapFloat dt = distanceTransform(" + name + ", int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE): Computes an unsigned distance transform. For distanceType and maskSize see "
      "http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#distancetransform").c_str());

  def("distanceTransformLabeled", &distanceTransformLabeledWrapper<T>, (boost::python::arg("distanceType") = (int)CV_DIST_L2, boost::python::arg("maskSize") = (int)CV_DIST_MASK_PRECISE),
      ("tuple(MapFloat dt, MapInt32 labels) = distanceTransformLabeled(" + name + ", int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE): Computes an unsigned distance transform and discretized Voronoi diagram. "
       "For distanceType and maskSize see http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#distancetransform").c_str());

  def("distanceTransformSigned", &signedDistanceTransformWrapper<T>, (boost::python::arg("distanceType") = (int)CV_DIST_L2, boost::python::arg("maskSize") = (int)CV_DIST_MASK_PRECISE),
      ("tuple(MapFloat dt, MapInt32 labels) = distanceTransformLabeled(" + name + ", int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE): Computes a signed distance transform and discretized Voronoi diagram. "
       "For distanceType and maskSize see http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#distancetransform").c_str());
}

void exportDistanceTransform()
{
  exportDistanceTransformT<bool>("MapBool");
  exportDistanceTransformT<uint8_t>("MapUint8");
  exportDistanceTransformT<uint16_t>("MapUint16");
  exportDistanceTransformT<uint32_t>("MapUint32");
  exportDistanceTransformT<uint64_t>("MapUint64");
  exportDistanceTransformT<float>("MapFloat");
  exportDistanceTransformT<double>("MapDouble");
  exportDistanceTransformT<OccupancyValue>("OccupancyGrid");
} /* void exportVoronoi() */
