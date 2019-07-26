/*
 * MathSupportPy.cpp
 *
 *  Created on: 14.07.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/MathSupport.hpp>

using namespace boost::python;
using namespace planning2d::math;

Eigen::Vector4d cubicInterpolatePchipCoefficientsWrapper(const Eigen::Vector4d p) {
  Eigen::Vector4d coeff;
  cubicInterpolatePchipCoefficients(p, coeff);
  return coeff;
}

namespace details {

template <int METHOD>
struct InterpolationMethodTraits {
  typedef Eigen::Matrix4d Matrix;
};

template <>
struct InterpolationMethodTraits<LINEAR> {
  typedef Eigen::Matrix2d Matrix;
};

} /* namespace details*/

template <int METHOD>
double interpolate2dWrapper(
    const Interpolator2d<METHOD>& interp,
    const typename ::details::InterpolationMethodTraits<METHOD>::Matrix p,
    const double x,
    const double y) {
  return interp.interpolate(p, x, y);
}

template <int METHOD>
Eigen::Vector2d gradient2dWrapper(
    const Interpolator2d<METHOD>& interp,
    const typename ::details::InterpolationMethodTraits<METHOD>::Matrix p,
    const double x,
    const double y) {
  return interp.gradient(p, x, y);
}


template <int METHOD>
void exportInterpolator2d(const std::string& name) {
  typedef Interpolator2d<METHOD> Interpolator;
  class_<Interpolator>(name.c_str(), init<>((name + "(): Constructor").c_str()))
    .def("interpolate", &interpolate2dWrapper<METHOD>)
    .def("gradient", &gradient2dWrapper<METHOD>)
  ;
}

void exportMathSupport() {

  enum_<InterpolationMethod>("InterpolationMethod")
    .value("LINEAR", InterpolationMethod::LINEAR)
    .value("CUBIC_CATMULL_ROM", InterpolationMethod::CUBIC_CATMULL_ROM)
    .value("CUBIC_PCHIP", InterpolationMethod::CUBIC_PCHIP)
  ;

  // Note: make_function_aux to wrap functions to be called by Eigen copy so that numpy_eigen works

  // linear interpolation
  def("linearInterpolate", boost::python::detail::make_function_aux([](const Eigen::Vector2d p, const double x){ return linearInterpolate(p, x); },
                                                                    boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector2d, const double>()));
  def("gradientLinearInterpolateX", boost::python::detail::make_function_aux([](const Eigen::Vector2d y){ return gradientLinearInterpolateX(y); },
                                                                             boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector2d>()));
  def("gradientLinearInterpolateP", &gradientLinearInterpolateP);

  // bilinear interpolation
  def("bilinearInterpolate", boost::python::detail::make_function_aux([](const Eigen::Matrix2d p, const double x, const double y){ return bilinearInterpolate(p, x, y); },
                                                                      boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Matrix2d, const double, const double>()));
  def("gradientBilinearInterpolate", boost::python::detail::make_function_aux([](const Eigen::Matrix2d p, const double x, const double y){ return gradientBilinearInterpolate(p, x, y); },
                                                                              boost::python::default_call_policies(), boost::mpl::vector<Eigen::Vector2d, const Eigen::Matrix2d, const double, const double>()));

  // cubic interpolation
  def("cubicInterpolate", boost::python::detail::make_function_aux([](const Eigen::Vector4d p, const double x){ return cubicInterpolate(p, x); },
                                                                   boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector4d, const double>()));
  def("gradientCubicInterpolateX", boost::python::detail::make_function_aux([](const Eigen::Vector4d p, const double x){ return gradientCubicInterpolateX(p, x); },
                                                                            boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector4d, const double>()));
  def("gradientCubicInterpolateP", &gradientCubicInterpolateP);

  def("cubicInterpolatePchipCoefficients", &cubicInterpolatePchipCoefficientsWrapper);
  def("cubicInterpolatePchip", boost::python::detail::make_function_aux([](const Eigen::Vector4d p, const double x){ return cubicInterpolatePchip(p, x); },
                                                                              boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector4d, const double>()));
  def("gradientCubicInterpolatePchipX", boost::python::detail::make_function_aux([](const Eigen::Vector4d p, const double x){ return gradientCubicInterpolatePchipX(p, x); },
                                                                                       boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Vector4d, const double>()));
  def("gradientCubicInterpolatePchipP", boost::python::detail::make_function_aux([](const Eigen::Vector4d p, const double x){ return gradientCubicInterpolatePchipP(p, x); },
                                                                                       boost::python::default_call_policies(), boost::mpl::vector<Eigen::Vector4d, const Eigen::Vector4d, const double>()));

  // bicubic interpolation
  def("bicubicInterpolate", boost::python::detail::make_function_aux([](const Eigen::Matrix4d p, const double x, const double y){ return bicubicInterpolate(p, x, y); },
                                                                     boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Matrix4d, const double, const double>()));
  def("bicubicInterpolatePchip", boost::python::detail::make_function_aux([](const Eigen::Matrix4d p, const double x, const double y){ return bicubicInterpolatePchip(p, x, y); },
                                                                                boost::python::default_call_policies(), boost::mpl::vector<double, const Eigen::Matrix4d, const double, const double>()));

  // helper class
  exportInterpolator2d<LINEAR>("Interpolator2d_Linear");
  exportInterpolator2d<CUBIC_CATMULL_ROM>("Interpolator2d_CubicCatmullRom");
  exportInterpolator2d<CUBIC_PCHIP>("Interpolator2d_CubicPchip");

} /* void exportMathSupport() */
