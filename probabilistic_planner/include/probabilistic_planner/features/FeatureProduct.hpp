/*
 * FeatureProduct.hpp
 *
 *  Created on: 04.04.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPRODUCT_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPRODUCT_HPP_

#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Exceptions.hpp>

#include "RawFeature.hpp"
#include "FeatureStatistics.hpp"
#include "FeatureContainer.hpp"

namespace prob_planner
{

/**
 * \class FeatureSingletonProduct
 * \brief A product of two singleton features (the product is build over integrands, not the product of two integrals)
 */
class FeatureSingletonProduct: public IntegratedSingletonFeature<FeatureSingletonProduct> {

 private:
  typedef IntegratedSingletonFeature<FeatureSingletonProduct> parent_t;

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonProduct);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_product";
  inline FeatureSingletonProduct(const RawFeatureSingleton::ConstPtr& lhs,
                                 const RawFeatureSingleton::ConstPtr& rhs,
                                 const OptAgentType& agentType, /*the agent types of the features are irrelevant*/
                                 const double weight,
                                 const double scalingFactor = 1.0,
                                 const bool activateScaling = true);
  inline FeatureSingletonProduct(const sm::value_store::ValueStore& vpt,
                                 const FeatureContainer& container);
  ~FeatureSingletonProduct() { }

  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonProduct() : parent_t() { }

  static std::string createName(const RawFeature::ConstPtr& lhs, const RawFeature::ConstPtr& rhs) {
    SM_ASSERT_NOTNULL(planning2d::InitializationException, lhs, "");
    SM_ASSERT_NOTNULL(planning2d::InitializationException, rhs, "");
    return lhs->name() == rhs->name() ? "singleton_auto_product." + lhs->name() : "singleton_cross_product." + lhs->name() + "." + rhs->name();
  }
 private:
  RawFeatureSingleton::ConstPtr _lhs; /// \brief left-hand side feature
  RawFeatureSingleton::ConstPtr _rhs; /// \brief right-hand side feature

};

FeatureSingletonProduct::FeatureSingletonProduct(const RawFeatureSingleton::ConstPtr& lhs,
                                                 const RawFeatureSingleton::ConstPtr& rhs,
                                                 const OptAgentType& agentType,
                                                 const double weight,
                                                 const double scalingFactor /*= 1.0*/,
                                                 const bool activateScaling /*= true*/)
      : parent_t(createName(lhs, rhs), agentType, scalingFactor, activateScaling), _lhs(lhs), _rhs(rhs)
  {
    SM_ASSERT_EQ(planning2d::InitializationException, _lhs->numWeights(), _rhs->numWeights(), "Incompatible dimensionalities " << _lhs->numWeights() << " and " << _rhs->numWeights());
    this->setWeight(0, weight);
  }

FeatureSingletonProduct::FeatureSingletonProduct(const sm::value_store::ValueStore& vpt,
                                                 const FeatureContainer& container)
    : parent_t("INVALID" /*will be set below*/, vpt) {
  const auto lhs = container.getFeature(vpt.getString("lhs").get());
  const auto rhs = container.getFeature(vpt.getString("rhs").get());
  SM_ASSERT_NOTNULL(planning2d::LookupException, lhs, "Feature " << vpt.getString("lhs").get() << " has to be in the feature container before constructing product feature");
  SM_ASSERT_NOTNULL(planning2d::LookupException, rhs, "Feature " << vpt.getString("rhs").get() << " has to be in the feature container before constructing product feature");
  _lhs = boost::dynamic_pointer_cast<RawFeatureSingleton>(lhs);
  _rhs = boost::dynamic_pointer_cast<RawFeatureSingleton>(rhs);
  SM_ASSERT_NOTNULL(planning2d::LookupException, _lhs, "Feature " << vpt.getString("lhs").get() << " has to be a singleton feature");
  SM_ASSERT_NOTNULL(planning2d::LookupException, _rhs, "Feature " << vpt.getString("rhs").get() << " has to be a singleton feature");
  this->name() = createName(_lhs, _rhs);
}

template <typename Return, typename OptAgent_>
inline Return FeatureSingletonProduct::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  return _lhs->getExpression(timestamp, agent) * _rhs->getExpression(timestamp, agent);
}

template<>
inline double FeatureSingletonProduct::evaluate<double, OptAgent>(const planning2d::Time& timestamp, const OptAgent& agent) const {
  return evaluate<aslam::backend::ScalarExpression, collectors::OptAgentExpressionWrapper>(timestamp, agent).evaluate();
}

inline FeatureSingletonProduct::Ptr multiply(const RawFeatureSingleton::ConstPtr& lhs,
                                             const RawFeatureSingleton::ConstPtr& rhs,
                                             const OptAgentType& agentType,
                                             const double weight,
                                             const double scalingFactor = 1.0,
                                             const bool activateScaling = true) {
  return FeatureSingletonProduct::Ptr(new FeatureSingletonProduct(lhs, rhs, agentType, weight, scalingFactor, activateScaling));
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPRODUCT_HPP_ */
