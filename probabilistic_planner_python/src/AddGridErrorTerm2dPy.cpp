/*
 * GridTestErrorTermPy.cpp
 *
 *  Created on: 05.01.2016
 *      Author: Ulrich Schwesinger
 */

// boost::python
#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

// aslam
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>
#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/ExpressionErrorTerm.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

// probabilistic_planner
#include <probabilistic_planner/features/GridExpressionNode.hpp>

using namespace boost::python;
using namespace prob_planner;
using namespace planning2d::grid;
using namespace aslam::backend;

template <int INTERPOLATION_METHOD, int EXTRAPOLATION_METHOD>
void addGridErrorTerm2d(planning2d::Map<float>::ConstPtr dt, const Eigen::Vector2d& dvVal, OptimizationProblem& problem)
{
  typedef GridExpressionNode<float,INTERPOLATION_METHOD,EXTRAPOLATION_METHOD> GridExprNode;
  boost::shared_ptr< DesignVariableGenericVector<2> > dv( new DesignVariableGenericVector<2>(dvVal));
  dv->setActive(true);
  dv->setBlockIndex(0);
  GenericMatrixExpression<2, 1> abscissaExpr(dv);
  auto expr = aslam::backend::inverseSigmoid(aslam::backend::ScalarExpression(typename GridExprNode::Ptr(new GridExprNode(abscissaExpr, dt))), 1.0, 1.0, 0.0);
  auto err = toScalarNonSquaredErrorTerm(expr);
  problem.addDesignVariable(dv);
  problem.addErrorTerm(err);
}

void exportAddGridErrorTerm2d() {

  def("addGridErrorTerm2d_linear_constant", addGridErrorTerm2d<InterpolationMethod::LINEAR,ExtrapolationMethod::CONSTANT>, "None = addGridErrorTerm2d_linear_constant(MapFloat map, array2d pos, OptimizationProblem problem): ");
  def("addGridErrorTerm2d_linear_linear", addGridErrorTerm2d<InterpolationMethod::LINEAR,ExtrapolationMethod::LINEAR>, "None = addGridErrorTerm2d_linear_linear(MapFloat map, array2d pos, OptimizationProblem problem): ");
  def("addGridErrorTerm2d_catmullrom_constant", addGridErrorTerm2d<InterpolationMethod::CUBIC_CATMULL_ROM,ExtrapolationMethod::CONSTANT>, "None = addGridErrorTerm2d_catmullrom_constant(MapFloat map, array2d pos, OptimizationProblem problem): ");
  def("addGridErrorTerm2d_catmullrom_linear", addGridErrorTerm2d<InterpolationMethod::CUBIC_CATMULL_ROM,ExtrapolationMethod::LINEAR>, "None = addGridErrorTerm2d_catmullrom_linear(MapFloat map, array2d pos, OptimizationProblem problem): ");
  def("addGridErrorTerm2d_pchip_constant", addGridErrorTerm2d<InterpolationMethod::CUBIC_PCHIP,ExtrapolationMethod::CONSTANT>, "None = addGridErrorTerm2d_pchip_constant(MapFloat map, array2d pos, OptimizationProblem problem): ");
  def("addGridErrorTerm2d_pchip_linear", addGridErrorTerm2d<InterpolationMethod::CUBIC_PCHIP,ExtrapolationMethod::LINEAR>, "None = addGridErrorTerm2d_pchip_linear(MapFloat map, array2d pos, OptimizationProblem problem): ");

} /* void exportGridTestErrorTerm() */
