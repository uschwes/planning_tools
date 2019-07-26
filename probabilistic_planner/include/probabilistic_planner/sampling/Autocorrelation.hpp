/*
 * Autocorrelation.hpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_SAMPLING_AUTOCORRELATION_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_SAMPLING_AUTOCORRELATION_HPP_

#include <vector>

#include <Eigen/Dense>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>

namespace prob_planner {
namespace sampling {

namespace details {

Eigen::MatrixXd toMatrix(const std::vector<ContinuousScene::Ptr>& samples);

} /* namespace details */

Eigen::MatrixXd computeAutocorrelation(const Eigen::MatrixXd& samples);
Eigen::RowVectorXd computeIntegratedAutocorrelationTime(const Eigen::MatrixXd& samples);
double computeEffectiveSampleSize(const Eigen::MatrixXd& samples);
double computeEffectiveSampleSize(const Eigen::MatrixXd& samples, double& integratedAutocorrelationTime);

Eigen::MatrixXd computeAutocorrelation(const std::vector<ContinuousScene::Ptr>& samples);
Eigen::RowVectorXd computeIntegratedAutocorrelationTime(const std::vector<ContinuousScene::Ptr>& samples);
double computeEffectiveSampleSize(const std::vector<ContinuousScene::Ptr>& samples);
double computeEffectiveSampleSize(const std::vector<ContinuousScene::Ptr>& samples, double& integratedAutocorrelationTime);

} /* namespace prob_planner */
} /* namespace sampling */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_SAMPLING_AUTOCORRELATION_HPP_ */
