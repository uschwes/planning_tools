/*
 * XFeature.hpp
 *
 *  Created on: Jul 23, 2015
 *      Author: pfmark
 */

#ifndef TEST_XPOSFEATURE_HPP_
#define TEST_XPOSFEATURE_HPP_

#include <planner_interfaces/Support.hpp>

#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>

#include <probabilistic_planner/features/FeatureStatistics.hpp>


namespace prob_planner {

/// \brief Feature that computes the position value of an agent's trajectory at a certain timestamp
class PosFeature : public IntegratedSingletonFeature<PosFeature> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(PosFeature);

 public:
  using IntegratedSingletonFeature<PosFeature>::evaluate;

  /// \brief Constructor: Also defines, for which kind of agent the feature is defined
  PosFeature(const OptAgentType& type, const double weight, const std::size_t index = 0) :
    IntegratedSingletonFeature<PosFeature>("position feature", type),
    _index(index) {

    assert(_index < 2);
    setWeight(0, weight);

  }
  ~PosFeature() { }

  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  PosFeature() : IntegratedSingletonFeature<PosFeature>("position feature", OptAgentType::PEDESTRIAN), _index(0) { setWeight(0, 1.0); }
  const std::size_t _index;
};


template <typename Return, typename OptAgent_>
Return PosFeature::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  return agent.trajectory().getPosition(timestamp)[_index];
}

} /* namespace prob_planner */


#endif /* TEST_XPOSFEATURE_HPP_ */
