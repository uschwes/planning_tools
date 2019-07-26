/*
 * FeatureContainerPy.hpp
 *
 *  Created on: 20.08.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_PYTHON_FEATURECONTAINERMODIFIABLE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_PYTHON_FEATURECONTAINERMODIFIABLE_HPP_

#include <probabilistic_planner/features/FeatureContainer.hpp>

class FeatureContainerModifiable : public prob_planner::FeatureContainer {

 public:
  using prob_planner::FeatureContainer::FeatureContainer;
  using FeatureContainer::push_back;
};

#endif /* INCLUDE_PROBABILISTIC_PLANNER_PYTHON_FEATURECONTAINERMODIFIABLE_HPP_ */
