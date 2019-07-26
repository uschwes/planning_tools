/*
 * RawFeature.hpp
 *
 *  Created on: Apr 9, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURE_HPP_

#include <sm/logging.hpp>
#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Exceptions.hpp>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/MatrixExpression.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/features/FeatureWeightScalarDesignVariable.hpp>

namespace prob_planner {

class RawFeature {

 public:
  typedef aslam::backend::OptimizationProblem OptimizationProblem;
  typedef boost::shared_ptr<OptimizationProblem> OptimizationProblemPtr;
  typedef FeatureWeightScalarDesignVariable DesignVariable;
  PLANNING_2D_POINTER_TYPEDEFS(RawFeature);

 public:
  RawFeature(const std::string& name,
             const OptAgentType& optAgentType,
             const std::size_t dimension,
             const double scalingFactor = 1.0,
             const bool activateScaling = true);
  RawFeature(const std::string& name, const sm::value_store::ValueStore& vpt, const std::size_t dimension);
  virtual ~RawFeature() { }

  std::string& name() { return _name; }
  const std::string& name() const { return _name; }
  std::string nameWithOptAgentType() const { return _name + "." + OptAgentTypeRegistry::getRegistry().toString(_optAgentType); }

  /// \brief Add the error terms comprising the negative log density formulation to an optimization (minimization) problem
  virtual void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const = 0;
  /// \brief Evaluate the feature value over a complete scene
  virtual Eigen::VectorXd evaluate(const ContinuousScene& /*scene*/) const { throw std::runtime_error(std::string("Not implemented in") + typeid(*this).name()); } //TODO remove this default implementation (make it pure virtual) and let all existing features implement it somehow

  /// \brief Adds the information of this feature to the value store object passed
  void save(sm::value_store::ExtendibleValueStore& vpt) const;

  OptAgentType getOptAgentType() const { return _optAgentType; }
  void setOptAgentType(const OptAgentType& optAgentType) { _optAgentType = optAgentType; }
  void setOptAgentType(const std::string& type) {  _optAgentType = OptAgentTypeRegistry::getRegistry().fromString(type); }

  std::size_t numWeights() const { return _weights.size(); }
  const std::vector<DesignVariable>& getWeights() const { return _weights; }
  inline double getWeight(const std::size_t i = 0) const;
  inline DesignVariable* getWeightAsDesignVariable(const std::size_t i);
  inline void setWeight(const std::size_t i, const double w);
  inline const Eigen::VectorXd& getCurrentWeightsVector() const;
  void setWeightsFromValueStore(const sm::value_store::ValueStore& vpt);

  inline std::size_t numActiveWeights() const;
  void activateForLearning(const bool activate);
  bool isActiveForLearning() const;

  /// \brief Allow/disallow negative feature weights
  void forbidNegativeWeights(bool f);

  const Eigen::VectorXd& getScalingFactor() const { return _scalingFactor; }
  inline double getScalingFactor(const std::size_t i) const;
  bool isScalingActive() const { return _isScalingActive; }
  void activateScaling(const bool activate) { _isScalingActive = activate; }

  /// \brief Scales the current output of the feature by \p s
  void scale(const Eigen::VectorXd& s);
  /// \brief Scales the current output of the i-th dimension of the feature by \p s
  void scale(const std::size_t i, const double s);

 protected:
  RawFeature() { }
  void setWeights(std::vector<DesignVariable>& featureWeights) { _weights = featureWeights; }
  void setScalingFactor(const Eigen::VectorXd& s) { _scalingFactor = s; }

 private:
  inline void setScalingFactor(const std::size_t i, const double s);
  void setScalingFactorFromValueStore(const sm::value_store::ValueStore& vpt);

 private:
  virtual void saveImpl(sm::value_store::ExtendibleValueStore& /*vpt*/) const { }

  std::string _name;
  OptAgentType _optAgentType = OptAgentType::UNKNOWN;
  std::vector<DesignVariable> _weights;
  mutable Eigen::VectorXd _weightsVector; /// \brief Simply to avoid memory allocation in getCurrentWeightsVector()
  Eigen::VectorXd _scalingFactor;
  bool _isScalingActive = true; /// \brief Whether or not a call to scale() will adapt the scaling

};


} /* end namespace prob_planner */

#include "impl/RawFeatureImpl.hpp"

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURE_HPP_ */

