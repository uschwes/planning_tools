/*
 * MathSupport.hpp
 *
 *  Created on: 06.01.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PLANNER_INTERFACES_MATHSUPPORT_HPP_
#define INCLUDE_PLANNER_INTERFACES_MATHSUPPORT_HPP_

#include <algorithm> // std::unique
#include <type_traits> // std::is_floating_point, std::is_integral
#include <vector>

#include <Eigen/Dense>

#include "Exceptions.hpp"
#include "Support.hpp"

namespace planning2d {

namespace math {

/// Power-2 function
template <typename T>
inline T square(const T& val);

/**
 * Computes the linear interpolation given four data points
 * @param p two equally spaced data points
 * @param x interpolation factor between 1st and 2nd point in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double linearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the gradient of the linearInterpolate() function in x-direction
 * @param p two equally spaced data points
 * @return Interpolated value
 */
template <typename Derived>
inline double gradientLinearInterpolateX(const Eigen::MatrixBase<Derived>& p);

/**
 * Computes the gradient of the linearInterpolate() function in the direction of the 2 points
 * @param x interpolation factor between 1st and 2nd point in interval [0,1]
 * @return Gradient in parameter direction
 */
inline Eigen::Vector2d gradientLinearInterpolateP(const double x);

/**
 * Computes the bilinear interpolation given 4 data points using linearInterpolate()
 * @param p 4 equally spaced data points
 * @param x interpolation factor between 1st and 2nd point in x-direction in interval [0,1]
 * @param y interpolation factor between 1st and 2nd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double bilinearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Computes the gradient of the bilinearInterpolate() function
 * @param p 4 equally spaced data points
 * @param x interpolation factor between 1st and 2nd point in x-direction in interval [0,1]
 * @param y interpolation factor between 1st and 2nd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline Eigen::Vector2d gradientBilinearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Computes the cubic interpolation given four data points with a Catmull-Rom spline
 * See http://www.paulinternet.nl/?page=bicubic
 * @param p four equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double cubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the gradient of the cubicInterpolate() function in x-direction
 * @param p four equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Gradient in x-direction
 */
template <typename Derived>
inline double gradientCubicInterpolateX(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the gradient of the cubicInterpolate() function in the direction of the 4 points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Gradient in parameter direction
 */
inline Eigen::Vector4d gradientCubicInterpolateP(const double x);

/**
 * Computes the cubic polynomial coefficients for PCHIP interpolation
 * @param[in] p four equally spaced data points
 * @param[in] x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @param[out] coeff Coefficients of the polynomial, i.e. f(x) = coeff[0] + coeff[1]*x + ...
 */
template <typename Derived1, typename Derived2>
inline void cubicInterpolatePchipCoefficients(const Eigen::MatrixBase<Derived1>& p, Eigen::MatrixBase<Derived2>& coeff);

/**
 * Computes the PCHIP interpolation given four data points
 * @param p four equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double cubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the gradient of the cubicInterpolatePchip() function in x-direction
 * @param p four equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Gradient in x-direction
 */
template <typename Derived>
inline double gradientCubicInterpolatePchipX(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the gradient of the cubicInterpolatePchip() function in the direction of the 4 points.
 * This gives how much the function value at point \p x changes in relation to a change of the 4 input points.
 * @param p four equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in interval [0,1]
 * @return Gradient w.r.t. points \p p
 */
template <typename Derived>
inline Eigen::Vector4d gradientCubicInterpolatePchipP(const Eigen::MatrixBase<Derived>& p, const double x);

/**
 * Computes the bicubic interpolation given 16 data points using cubicInterpolate()
 * See http://www.paulinternet.nl/?page=bicubic
 * @param p 16 equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
 * @param y interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double bicubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Computes the gradient of the bicubicInterpolate() function
 * @param p 16 equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
 * @param y interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline Eigen::Vector2d gradientBicubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Computes the constrained bicubic interpolation given 16 data points using cubicInterpolatePchip()
 * @param p 16 equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
 * @param y interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline double bicubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Computes the gradient of the bicubicInterpolatePchip() function
 * @param p 16 equally spaced data points
 * @param x interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
 * @param y interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
 * @return Interpolated value
 */
template <typename Derived>
inline Eigen::Vector2d gradientBicubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x, const double y);

/**
 * Returns a list of most evenly spaced values in the range [\p start, \p end].
 * Output values are guaranteed to include \p start and \p end.
 * If Scalar type is not a floating point type and \p makeUnique is false, values might contain non-unique ones.
 * If Scalar type is not a floating point type and \p makeUnique is true, output vector might contain less than
 * \p numVals values.
 * @param[in]  start start value, guaranteed to be included in \p vals
 * @param[in]  end end value, guaranteed to be included in \p vals
 * @param[in]  numVals Desired number of output values
 * @param[out] vals Output array
 * @param[in]  makeUnique Only relevant for
 */
template <typename Scalar = double, template <typename, typename...> class Container = std::vector>
void linspace(const Scalar start, const Scalar end, const std::size_t numVals, Container<Scalar>& vals, const bool makeUnique = true);

/**
 * @enum InterpolationMethod
 * LINEAR: Linear interpolation
 * CUBIC_CATMULL_ROM: Cubic interpolation with Catmull-Rom splines
 * CUBIC_PCHIP: Piecewise cubic Hermite interpolation polynomial
 */
enum InterpolationMethod { LINEAR, CUBIC_CATMULL_ROM, CUBIC_PCHIP };

/// Traits for interpolation kernel
template <int INTERPOLATION_METHOD>
struct InterpolationKernelTraits {
  static_assert(INTERPOLATION_METHOD == CUBIC_CATMULL_ROM || INTERPOLATION_METHOD == CUBIC_PCHIP, "");
  static constexpr int RowsAndCols = 4;
};

template <>
struct InterpolationKernelTraits<InterpolationMethod::LINEAR> {
  static constexpr int RowsAndCols = 2;
};

/**
 * @class Interpolator1d
 * Helper class for one-dimensional interpolation
 * @tparam METHOD Interpolation method
 */
template <int METHOD = LINEAR>
class Interpolator1d {
 public:

  Interpolator1d() = default;

  /**
   * Returns the interpolated value at point \p x
   * @param p Vector of points of size InterpolationKernelTraits::RowsAndCols
   * @param x Interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
   * @return Interpolated value
   */
  template <typename Derived>
  static inline double interpolate(
      const Eigen::MatrixBase<Derived>& p,
      const double x);

  /**
   * Returns the gradient of the interpolated function at point \p x
   * @param p Vector of points of size InterpolationKernelTraits::RowsAndCols
   * @param x Interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
   * @param y Interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
   * @return Gradient of interpolated function at \p x
   */
  template <typename Derived>
  static inline double gradient(
      const Eigen::MatrixBase<Derived>& p,
      const double x);
};

/**
 * @class Interpolator2d
 * Helper class for two-dimensional interpolation
 * @tparam METHOD Interpolation method
 */
template <int METHOD = LINEAR>
class Interpolator2d {
 public:

  Interpolator2d() = default;

  /**
   * Returns the interpolated value at point \p x
   * @param p Square matrix of points of size InterpolationKernelTraits::RowsAndCols
   * @param x Interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
   * @param y Interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
   * @return Interpolated value
   */
  template <typename Derived>
  static inline double interpolate(
      const Eigen::MatrixBase<Derived>& p,
      const double x,
      const double y);

  /**
   * Returns the gradient of the interpolated function at point \p x
   * @param p Square matrix of points of size InterpolationKernelTraits::RowsAndCols
   * @param x Interpolation factor between 2nd and 3rd point in x-direction in interval [0,1]
   * @param y Interpolation factor between 2nd and 3rd point in y-direction in interval [0,1]
   * @return Gradient of interpolated function at (\p x, \p y)
   */
  template <typename Derived>
  static inline Eigen::Vector2d gradient(
      const Eigen::MatrixBase<Derived>& p,
      const double x,
      const double y);
};

} /* namespace math */

} /* namespace planning2d */

#include "implementation/MathSupportImplementation.hpp"

#endif /* INCLUDE_PLANNER_INTERFACES_MATHSUPPORT_HPP_ */
