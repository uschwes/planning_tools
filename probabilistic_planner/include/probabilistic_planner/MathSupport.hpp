/*
 * MathSupport.hpp
 *
 *  Created on: 15.12.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_MATHSUPPORT_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_MATHSUPPORT_HPP_

// standard
#include <cmath>

namespace prob_planner {

namespace math {

  inline double inverseSigmoid(const double x, const double height, const double scale, const double shift)
  {
    return height * 0.5 * (1. + tanh( - scale*(x - shift) * 0.5));
  }

  inline double acosSquared(const double arg)
  {
    const auto tmp = acos(arg);
    return tmp * tmp;
  }
  
  inline double piecewiseExpression(const double e1, const double e2, std::function<bool()> useFirst) {
    if (useFirst())
      return e1;
    return e2;
  }
  
  inline double powerExpression(double arg, int k) {
    return pow(arg, k);
  }

} /* namespace math */

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_MATHSUPPORT_HPP_ */
