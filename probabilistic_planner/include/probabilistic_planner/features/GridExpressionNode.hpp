/*
 * GridExpressionNode.hpp
 *
 *  Created on: 14.12.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_GRIDEXPRESSIONNODE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_GRIDEXPRESSIONNODE_HPP_

// aslam
#include <aslam/backend/GenericScalarExpression.hpp>

// planner interfaces
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>

// self
#include <probabilistic_planner/Support.hpp>

namespace prob_planner {

/**
 * @class GridExpressionNode
 * @brief Perform scalar operation on the values stored in a 2D grid.
 * @tparam T Scalar type stored in the grid
 * @tparam INTERPOLATION_METHOD Type of interpolation, choose cubic (or higher if available) for smooth gradients.
 * @tparam EXTRAPOLATION_METHOD Type of extrapolation, important if query outside map boundaries.
 *                              ExtrapolationMethod::NONE will throw upon out of bounds access!
 */
template <typename T,
          int INTERPOLATION_METHOD = planning2d::grid::InterpolationMethod::CUBIC_CATMULL_ROM,
          int EXTRAPOLATION_METHOD = planning2d::grid::ExtrapolationMethod::CONSTANT>
class GridExpressionNode : public aslam::backend::ScalarExpressionNode
{
 private:
  typedef GridExpressionNode<T,INTERPOLATION_METHOD,EXTRAPOLATION_METHOD> this_t;

 public:
  PLANNING_2D_POINTER_TYPEDEFS(this_t);

 public:
  GridExpressionNode(const aslam::backend::GenericMatrixExpression<2,1>& abscissaExpr,
                     typename planning2d::Map<T>::ConstPtr grid)
      : _abscissaExpr(abscissaExpr), _grid(grid) { }
  ~GridExpressionNode() { }

 protected:
  virtual double evaluateImplementation() const override {
    planning2d::Position2d pos = planning2d::Position2d(_abscissaExpr.evaluate());
    auto index = _grid->toInterpolatedIndex(pos);
    try
    {
      return _grid->template atInterpolated<INTERPOLATION_METHOD, EXTRAPOLATION_METHOD>(index);
    }
    catch (const planning2d::OutOfBoundAccessException& e)
    {
      SM_FATAL_STREAM("Grid accessed outside the boundaries at position " << pos << " and interpolated index " << index <<
                      ". Specify an extrapolation method different than NONE to deal with this case");
      throw;
    }
  }
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const override {
    if (fabs(_grid->getOrigin().yaw()) > 1e-12)
      _abscissaExpr.evaluateJacobians(outJacobians.apply(this->getNumericDerivative()*_grid->getTransformation().rotation()));
    else // optimized version if yaw is zero
      _abscissaExpr.evaluateJacobians(outJacobians.apply(this->getNumericDerivative()));
  }
  virtual void getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const override {
    _abscissaExpr.getDesignVariables(designVariables);
  }

 private:
  /// \brief Returns the (numeric) derivative of the grid values w.r.t. the abscissa
  Eigen::RowVector2d getNumericDerivative() const {
    ::prob_planner::Timer timer(std::string("GridExpressionNode: ") + __FUNCTION__, false);
    auto pos = planning2d::Position2d(_abscissaExpr.evaluate());
    auto index = _grid->toInterpolatedIndex(pos);
    try
    {
      return _grid->template gradientInterpolated<INTERPOLATION_METHOD, EXTRAPOLATION_METHOD>(index);
    } catch (const planning2d::OutOfBoundAccessException& e)
    {
      SM_FATAL_STREAM("Grid accessed outside the boundaries at position " << pos << " and interpolated index " << index <<
                      ". Specify an extrapolation method different than NONE to deal with this case");
      throw;
    }
  }
 private:
  aslam::backend::GenericMatrixExpression<2,1> _abscissaExpr; /// \brief The 2D abscissa/index of the grid
  typename planning2d::Map<T>::ConstPtr _grid;                /// \brief The grid itself
};

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_GRIDEXPRESSIONNODE_HPP_ */
