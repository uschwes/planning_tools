/*
 * OverlappingTimeAgentsPairwiseFeature.hpp
 *
 *  Created on: Jul 24, 2015
 *      Author: pfmark
 */

#ifndef TEST_OVERLAPPINGTIMEAGENTSPAIRWISE_HPP_
#define TEST_OVERLAPPINGTIMEAGENTSPAIRWISE_HPP_

#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Time.hpp>

#include <probabilistic_planner/features/FeatureStatistics.hpp>

namespace prob_planner {

class OverlappingTimeAgentsPairwiseFeature: public IntegratedPairwiseFeature<OverlappingTimeAgentsPairwiseFeature> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(OverlappingTimeAgentsPairwiseFeature);

 public:
  using IntegratedPairwiseFeature<OverlappingTimeAgentsPairwiseFeature>::evaluate;

  /// \brief Constructor: Also defines, for which kind of agent the feature is defined
  OverlappingTimeAgentsPairwiseFeature(const OptAgentType& type, const double weight = 1.0) : IntegratedPairwiseFeature<OverlappingTimeAgentsPairwiseFeature>("overlapping time pairwise", type) { setWeight(0, weight); }
  OverlappingTimeAgentsPairwiseFeature(const sm::value_store::ValueStore& vpt) : IntegratedPairwiseFeature<OverlappingTimeAgentsPairwiseFeature>("overlapping time pairwise", vpt) { }
  ~OverlappingTimeAgentsPairwiseFeature() { }

  /// \brief Evaluate the feature at a given timestamp for an agent pair
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent0, const OptAgent_ & agent1) const;

 private:
  OverlappingTimeAgentsPairwiseFeature() : IntegratedPairwiseFeature<OverlappingTimeAgentsPairwiseFeature>("dummy feature pairwise", OptAgentType::UNKNOWN) { setWeight(0, 1.0); }

};



template <typename Return, typename OptAgent_>
Return OverlappingTimeAgentsPairwiseFeature::evaluate(const planning2d::Time& /*timestamp*/, const OptAgent_& /*agent0*/, const OptAgent_ & /*agent1*/) const {
  return 1.0;
}

} /* namespace prob_planner */



#endif /* TEST_OVERLAPPINGTIMEAGENTSPAIRWISE_HPP_ */
