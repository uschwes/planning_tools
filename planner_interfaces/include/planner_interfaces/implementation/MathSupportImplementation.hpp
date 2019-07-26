/*
 * MathSupportImplementation.hpp
 *
 *  Created on: 27.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_MATHSUPPORTIMPLEMENTATION_HPP_
#define INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_MATHSUPPORTIMPLEMENTATION_HPP_


#include <algorithm> // std::unique
#include <type_traits> // std::is_floating_point
#include <vector>

#include <Eigen/Dense>

#include <planner_interfaces/Exceptions.hpp>
#include <planner_interfaces/Support.hpp>

namespace planning2d {

namespace math {

namespace details {

template <typename Scalar>
struct InterpolationTypeTraits {
  typedef Scalar value_type;
};

template <>
struct InterpolationTypeTraits<float> {
  typedef double value_type;
};

} /* namespace details */

template <typename T>
inline T square(const T& val) { return val*val; }

template <typename Derived>
inline double linearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x) {
  static_assert(Derived::SizeAtCompileTime == 2, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");
  typedef typename details::InterpolationTypeTraits<typename Derived::Scalar>::value_type Scalar;
  Timer timer("MathSupport: linearInterpolate", false);
  Eigen::Matrix<Scalar, 2, 1> p_ = p.template cast<Scalar>();
  return (p_[1] - p_[0])*x + p_[0];
}

template <typename Derived>
inline double gradientLinearInterpolateX(const Eigen::MatrixBase<Derived>& p) {
  static_assert(Derived::SizeAtCompileTime == 2, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");
  typedef typename details::InterpolationTypeTraits<typename Derived::Scalar>::value_type Scalar;
  Timer timer("MathSupport: gradientLinearInterpolateX", false);
  Eigen::Matrix<Scalar, 2, 1> p_ = p.template cast<Scalar>();
  return p_[1] - p_[0];
}

inline Eigen::Vector2d gradientLinearInterpolateP(const double x) {
  Timer timer("MathSupport: gradientLinearInterpolateP", false);
  Eigen::Vector2d grad;
  grad[0] = -x + 1.0; // dy/dp[0]
  grad[1] = x; // dy/dp[1]
  return grad;
}

template <typename Derived>
inline double bilinearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {
  static_assert(Derived::RowsAtCompileTime == 2 && Derived::ColsAtCompileTime == 2, "Invalid shape of points matrix");
  Timer timer("MathSupport: bilinearInterpolate", false);
  Eigen::Vector2d vals;
  for (int i=0; i<2; i++)
    vals[i] = math::linearInterpolate(p.col(i), y);
  return math::linearInterpolate(vals, x);
}

template <typename Derived>
inline Eigen::Vector2d gradientBilinearInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {

  static_assert(Derived::RowsAtCompileTime == 2 && Derived::ColsAtCompileTime == 2, "");
  Timer timer("MathSupport: gradientBilinearInterpolate", false);

  Eigen::Vector2d grad;

  Eigen::Vector2d valsX;
  for (int i=0; i<2; i++)
    valsX[i] = math::linearInterpolate(p.col(i), y);
  grad.x() = math::gradientLinearInterpolateX(valsX);

  Eigen::RowVector2d gradsY;
  for (int i=0; i<2; i++)
    gradsY[i] = math::gradientLinearInterpolateX(p.col(i));
  Eigen::Vector2d gradsP = math::gradientLinearInterpolateP(x);
  grad.y() = gradsY*gradsP;

  return grad;
}

template <typename Derived>
inline double cubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x) {
  static_assert(Derived::SizeAtCompileTime == 4, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");
  SM_ASSERT_GE_LE(planning2d::FunctionInputException, x, 0.0, 1.0, "");
  typedef typename details::InterpolationTypeTraits<typename Derived::Scalar>::value_type Scalar;
  Timer timer("MathSupport: cubicInterpolate", false);
  Eigen::Matrix<Scalar, 4, 1> p_ = p.template cast<Scalar>();
  return p_[1] + 0.5 * x*(p_[2] - p_[0] + x*(2.0*p_[0] - 5.0*p_[1] + 4.0*p_[2] - p_[3] + x*(3.0*(p_[1] - p_[2]) + p_[3] - p_[0])));
}

template <typename Derived>
inline double gradientCubicInterpolateX(const Eigen::MatrixBase<Derived>& p, const double x) {
  static_assert(Derived::SizeAtCompileTime == 4, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");
  SM_ASSERT_GE_LE(planning2d::FunctionInputException, x, 0.0, 1.0, "");
  typedef typename details::InterpolationTypeTraits<typename Derived::Scalar>::value_type Scalar;
  Timer timer("MathSupport: gradientCubicInterpolateX", false);
  Eigen::Matrix<Scalar, 4, 1> p_ = p.template cast<Scalar>();
  return 0.5*(p_[2] - p_[0]) + math::square(x)*(-1.5*p_[0] + 4.5*p_[1] - 4.5*p_[2] + 1.5*p_[3]) + x*(2.0*p_[0] - 5.0*p_[1] + 4.0*p_[2] - 1.0*p_[3]);
}

inline Eigen::Vector4d gradientCubicInterpolateP(const double x) {
  SM_ASSERT_GE_LE(planning2d::FunctionInputException, x, 0.0, 1.0, "");
  Timer timer("MathSupport: gradientCubicInterpolateP", false);
  Eigen::Vector4d grad;
  const double x2 = math::square(x);
  grad[0] = -0.5*x*(x*(x - 2.0) + 1.0);
  grad[1] = x2*(1.5*x - 2.5) + 1.0;
  grad[2] = x*(-1.5*x2 + 2.0*x + 0.5);
  grad[3] = 0.5*x2*(x - 1.0);
  return grad;
}

template <typename Derived1, typename Derived2>
inline void cubicInterpolatePchipCoefficients(const Eigen::MatrixBase<Derived1>& p, Eigen::MatrixBase<Derived2>& coeff) {
  static_assert(Derived1::SizeAtCompileTime == 4, "");
  static_assert(Derived1::RowsAtCompileTime == 1 || Derived1::ColsAtCompileTime == 1, "");
  static_assert(Derived2::SizeAtCompileTime == 4, "");
  static_assert(Derived2::RowsAtCompileTime == 1 || Derived2::ColsAtCompileTime == 1, "");

  const auto delta = (p.tail(3) - p.head(3)).eval();
  const double num1 = delta[1]*delta[0];
  const double num2 = delta[2]*delta[1];
  const double dy1 = num1 > 0. ? 2.*num1 / (p[2] - p[0]) : 0.; // derivative at 1st point
  const double dy2 = num2 > 0. ? 2.*num2 / (p[3] - p[1]) : 0.; // derivative at 2nd point

  coeff[0] = p[1];
  coeff[1] = dy1;
  coeff[2] = -(2.*dy1 + dy2 + 3.*(p[1] - p[2]));
  coeff[3] = dy2 + dy1 + 2.*(p[1] - p[2]);
}

template <typename Derived>
inline double cubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x) {
  static_assert(Derived::SizeAtCompileTime == 4, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");
  SM_ASSERT_GE_LE(planning2d::FunctionInputException, x, 0.0, 1.0, "");

  Eigen::Vector4d coeff;
  cubicInterpolatePchipCoefficients(p, coeff);

  const double xsqr = math::square(x);
  return coeff[0] + coeff[1]*x + coeff[2]*xsqr + coeff[3]*x*xsqr;
}

template <typename Derived>
inline double gradientCubicInterpolatePchipX(const Eigen::MatrixBase<Derived>& p, const double x) {
  Eigen::Vector4d coeff;
  cubicInterpolatePchipCoefficients(p, coeff);
  const double xsqr = math::square(x);
  return coeff[1] + 2.*coeff[2]*x + 3.*coeff[3]*xsqr;
}

template <typename Derived>
inline Eigen::Vector4d gradientCubicInterpolatePchipP(const Eigen::MatrixBase<Derived>& p, const double x) {
  static_assert(Derived::SizeAtCompileTime == 4, "");
  static_assert(Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1, "");

  // Remember:
  //  a = p[1];
  //  b = dy1;
  //  c = -2.*dy1 - dy2 + 3.*p[2] - 3*p[1];
  //  d = dy2 + dy1 + 2.*p[1] - 2*p[2];

  const auto delta = (p.tail(3) - p.head(3)).eval(); // [y1 - y0, y2 - y1, y3 - y2]
  const double num1 = delta[1]*delta[0]; // (y2 - y1)*(y1 - y2)
  const double num2 = delta[2]*delta[1]; // (y3 - y2)*(y2 - y1)

  // Compute derivatives of coefficients with respect to points
  Eigen::Matrix4d M; // Matrix of partial derivatives of coefficients w.r.t. the points

  const double x2 = math::square(x);
  const double x3 = x2*x;
  Eigen::Vector4d xx = (Eigen::Vector4d() << 1., x, x2, x3).finished();
  const double d20 = p[2] - p[0];
  const double d20inv = 1./d20;
  const double d20sqrinv = square(d20inv);
  const double d31 = p[3] - p[1];
  const double d31inv = 1./d31;
  const double d31sqrinv = math::square(d31inv);

  // Here we compute the derivatives of the slope at the boundary points w.r.t. the points
  double ddy1dp0, ddy1dp1, ddy1dp2, ddy1dp3, ddy2dp0, ddy2dp1, ddy2dp2, ddy2dp3;
  if (num1 > 0) {
    ddy1dp0 = 2.0*(num1 - d20*delta[1])*d20sqrinv;
    ddy1dp1 = 2.0*(p[0] - 2.0*p[1] + p[2])*d20inv;
    ddy1dp2 = 2.0*square(delta[0])*d20sqrinv;
    ddy1dp3 = 0.0;
  } else {
    ddy1dp0 = ddy1dp1 = ddy1dp2 = ddy1dp3 = 0.0;
  }
  if (num2 > 0) {
    ddy2dp0 = 0.0;
    ddy2dp1 = 2.0*(num2 - d31*delta[2])*d31sqrinv;
    ddy2dp2 = 2.0*(p[1] - 2.0*p[2] + p[3])*d31inv;
    ddy2dp3 = 2.0*square(delta[1])*d31sqrinv;
  } else {
    ddy2dp0 = ddy2dp1 = ddy2dp2 = ddy2dp3 = 0.0;
  }

  // Now we use the definition of the coefficients to build up their derivatives w.r.t. the points
  const double dadp0 = 0.;
  const double dadp1 = 1.;
  const double dadp2 = 0.;
  const double dadp3 = 0.;
  M.row(0) << dadp0, dadp1, dadp2, dadp3;

  const double dbdp0 = ddy1dp0;
  const double dbdp1 = ddy1dp1;
  const double dbdp2 = ddy1dp2;
  const double dbdp3 = ddy1dp3;
  M.row(1) << dbdp0, dbdp1, dbdp2, dbdp3;

  const double dcdp0 = -2.*ddy1dp0 - ddy2dp0;
  const double dcdp1 = -2.*ddy1dp1 - ddy2dp1 - 3.;
  const double dcdp2 = -2.*ddy1dp2 - ddy2dp2 + 3.;
  const double dcdp3 = -2.*ddy1dp3 - ddy2dp3;
  M.row(2) << dcdp0, dcdp1, dcdp2, dcdp3;

  const double dddp0 = ddy2dp0 + ddy1dp0;
  const double dddp1 = ddy2dp1 + ddy1dp1 + 2.;
  const double dddp2 = ddy2dp2 + ddy1dp2 - 2.;
  const double dddp3 = ddy2dp3 + ddy1dp3;
  M.row(3) << dddp0, dddp1, dddp2, dddp3;

  // Now we basically do df/dp[i] = 1.*da/dp[i] + x*db/dp[i] + x^2*dc/dp[i] + x^3*dd/dp[i]
  return M.transpose() * xx;
}

template <typename Derived>
inline double bicubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {
  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4, "");
  Timer timer("MathSupport: bicubicInterpolate", false);
  Eigen::Vector4d vals;
  for (int i=0; i<4; i++)
    vals[i] = math::cubicInterpolate(p.col(i), y);
  return math::cubicInterpolate(vals, x);
}

template <typename Derived>
inline Eigen::Vector2d gradientBicubicInterpolate(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {

  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4, "");
  Timer timer("MathSupport: gradientBicubicInterpolate", false);

  Eigen::Vector2d grad;

  Eigen::Vector4d valsX;
  for (int i=0; i<4; i++)
    valsX[i] = math::cubicInterpolate(p.col(i), y);
  grad.x() = math::gradientCubicInterpolateX(valsX, x);

  Eigen::RowVector4d gradsY;
  for (int i=0; i<4; i++)
    gradsY[i] = math::gradientCubicInterpolateX(p.col(i), y);
  Eigen::Vector4d gradsP = math::gradientCubicInterpolateP(x);
  grad.y() = gradsY*gradsP;

  return grad;
}

template <typename Derived>
inline double bicubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {
  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4, "");
  Timer timer("MathSupport: bicubicInterpolatePchip", false);
  Eigen::Vector4d vals;
  for (int i=0; i<4; i++) {
    vals[i] = math::cubicInterpolatePchip(p.col(i), y);
  }
  return math::cubicInterpolatePchip(vals, x);
}

template <typename Derived>
inline Eigen::Vector2d gradientBicubicInterpolatePchip(const Eigen::MatrixBase<Derived>& p, const double x, const double y) {

  static_assert(Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4, "");
  Timer timer("MathSupport: gradientBicubicInterpolate", false);

  Eigen::Vector2d grad;

  Eigen::Vector4d valsX;
  for (int i=0; i<4; i++)
    valsX[i] = math::cubicInterpolatePchip(p.col(i), y);
  grad.x() = math::gradientCubicInterpolatePchipX(valsX, x);

  Eigen::RowVector4d gradsY;
  for (int i=0; i<4; i++)
    gradsY[i] = math::gradientCubicInterpolatePchipX(p.col(i), y);
  const auto gradsP = math::gradientCubicInterpolatePchipP(valsX, x);
  grad.y() = gradsY*gradsP;

  return grad;
}

template <typename Scalar = double, template <typename, typename...> class Container = std::vector>
void linspace(const Scalar start, const Scalar end, const std::size_t numVals, Container<Scalar>& vals, const bool makeUnique = true) {

  vals.clear();

  if (numVals == 0) {
    return;
  } else if (numVals == 1 || start == end) {
    vals.push_back(start);
    return;
  }

  double delta = static_cast<double>(end - start)/(numVals - 1);
  for (std::size_t i=0; i<numVals - 1; ++i)
    vals.push_back(static_cast<Scalar>(start + i*delta));
  vals.push_back(end);
  if (makeUnique && !std::is_floating_point<Scalar>::value)
    std::unique(vals.begin(), vals.end());
}

template <int METHOD>
template <typename Derived>
inline double Interpolator1d<METHOD>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return linearInterpolate(p, x);
}

template <int METHOD>
template <typename Derived>
inline double Interpolator1d<METHOD>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return gradientBilinearInterpolate(p, x);
}

template <>
template <typename Derived>
inline double Interpolator1d<CUBIC_CATMULL_ROM>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return cubicInterpolate(p, x);
}

template <>
template <typename Derived>
inline double Interpolator1d<CUBIC_PCHIP>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return cubicInterpolatePchip(p, x);
}

template <>
template <typename Derived>
inline double Interpolator1d<CUBIC_CATMULL_ROM>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return gradientCubicInterpolateX(p, x);
}

template <>
template <typename Derived>
inline double Interpolator1d<CUBIC_PCHIP>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x) {
  return gradientCubicInterpolatePchipX(p, x);
}

template <int METHOD>
template <typename Derived>
inline double Interpolator2d<METHOD>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return bilinearInterpolate(p, x, y);
}

template <int METHOD>
template <typename Derived>
inline Eigen::Vector2d Interpolator2d<METHOD>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return gradientBilinearInterpolate(p, x, y);
}

template <>
template <typename Derived>
inline double Interpolator2d<CUBIC_CATMULL_ROM>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return bicubicInterpolate(p, x, y);
}

template <>
template <typename Derived>
inline double Interpolator2d<CUBIC_PCHIP>::interpolate(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return bicubicInterpolatePchip(p, x, y);
}

template <>
template <typename Derived>
inline Eigen::Vector2d Interpolator2d<CUBIC_CATMULL_ROM>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return gradientBicubicInterpolate(p, x, y);
}

template <>
template <typename Derived>
inline Eigen::Vector2d Interpolator2d<CUBIC_PCHIP>::gradient(
    const Eigen::MatrixBase<Derived>& p,
    const double x,
    const double y) {
  return gradientBicubicInterpolatePchip(p, x, y);
}

extern template void linspace(const double start, const double end, const std::size_t numVals, std::vector<double>& vals, const bool makeUnique /*= true*/);
extern template void linspace(const int64_t start, const int64_t end, const std::size_t numVals, std::vector<int64_t>& vals, const bool makeUnique /*= true*/);

} /* namespace math */

} /* namespace planning2d */


#endif /* INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_MATHSUPPORTIMPLEMENTATION_HPP_ */
