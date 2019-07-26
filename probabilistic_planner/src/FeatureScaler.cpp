/*
 * FeatureScaler.cpp
 *
 *  Created on: 23.09.2015
 *      Author: Ulrich Schwesinger
 */

#include <probabilistic_planner/FeatureScaler.hpp>

using namespace std;

namespace prob_planner {

void scale(RawFeature::Ptr feature,
           const std::vector<ContinuousScene::Ptr>& demonstrations) {

  const size_t numDemonstrations = demonstrations.size();

  // compute statistics
  Eigen::MatrixXd fvals(feature->numWeights(), numDemonstrations);

  for (size_t i=0; i<numDemonstrations; i++)
    fvals.col(i) = feature->evaluate(*demonstrations.at(i));

  Eigen::VectorXd max = fvals.rowwise().maxCoeff();
  max = (max.array() < 1e-12).select(1e-12, max); // avoid division by zero
  feature->scale(max.cwiseInverse());
}

void scale(FeatureContainer& features,
           const std::vector<ContinuousScene::Ptr>& demonstrations) {

  for (auto& f : features.getContainer())
    scale(f, demonstrations);

}

}


