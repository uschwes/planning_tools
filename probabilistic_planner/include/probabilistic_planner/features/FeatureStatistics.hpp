/*
 * FeatureStatistics.hpp
 *
 *  Created on: May 5, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESTATISTICS_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESTATISTICS_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <sm/logging.hpp>

#include <aslam/backend/MatrixExpression.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include <planner_interfaces/State.hpp>

#include <probabilistic_planner/Support.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include <probabilistic_planner/state_representation/Trajectory.hpp>
#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/features/RawFeatureErrorTerm.hpp>
#include <probabilistic_planner/features/RawFeatureSingleton.hpp>
#include <probabilistic_planner/features/RawFeaturePairwise.hpp>

namespace prob_planner {

/* Set of templated functions that can be called by the features themselves */

template<typename FeatureType>
void integrateSingletonFeature(const FeatureType& feature,
                               const ContinuousScene& scene,
                               RawFeature::OptimizationProblem& optimizationProblem) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: integrateSingletonFeature", false);

  // pre-computations
  const planning2d::Duration timestep(0.1); // TODO: where to get this from?
  const planning2d::Time tmin = scene.getMinTime();
  const planning2d::Time tmax = scene.getMaxTime();

  for (auto t=tmin; t<tmax; t+=timestep) {
    const auto& agents = feature.getSuitableAgents(t, scene);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << ": Got " << agents.size() << " suitable agents for feature \"" << feature.name() << "\" for timestamp " << t.format(Formatter(SEC,2)) << ".");
    sumOverAgents(feature, agents, t, scene, optimizationProblem, timestep);  // iterate through all agents at current timestep
  }
}

template<typename FeatureType>
void integratePairwiseFeature(const FeatureType& feature,
                              const ContinuousScene& scene,
                              RawFeature::OptimizationProblem& optimizationProblem) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: integratePairwiseFeature", false);

  // pre-computations
  const planning2d::Duration timestep(0.1); // TODO: where to get this from?
  const planning2d::Time tmin = scene.getMinTime();
  const planning2d::Time tmax = scene.getMaxTime();

  for (auto t=tmin; t<tmax; t+=timestep) {
    const auto& agentPairs = feature.getSuitableAgentPairs(t, scene);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << ": Got " << agentPairs.size() << " suitable agent pairs for feature \"" << feature.name() << "\" for timestamp " << t.format(Formatter(SEC,2)) << ".");
    sumOverAgents(feature, agentPairs, t, scene, optimizationProblem, timestep); // iterate through all agents at current timestep
  }
}

namespace statistics {
using aslam::backend::GenericMatrixExpression;

class NopOptAgenWrapperFunctor {
  const OptAgent & operator () (const OptAgent & op) {
    return op;
  }
};

template <int r, int c>
std::ostream & operator << (std::ostream & os, const GenericMatrixExpression<r, c> & m) {
  os << m.evaluate();
  return os;
}

template<typename FeatureType, typename ValueCollector>
void sumOverAgents(const FeatureType& feature,
                   const std::vector< OptAgent::ConstRef >& agents,
                   const planning2d::Time& timestamp,
                   ValueCollector& valueCollector,
                   double factor) {

  using namespace planning2d::time;
  for (auto& agent : agents) {
    auto value = feature.template evaluate<typename ValueCollector::ValueType>(timestamp, valueCollector.wrap(agent));
    valueCollector.add(feature, value, factor, timestamp);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                            "Added error term at timestamp " << timestamp.format(Formatter(SEC,2)) << " for agent " << agent.get().getId() << " with value " << value);
  }
}


template<typename FeatureType, typename ValueCollector>
void sumOverAgents(const FeatureType& feature,
                   const std::vector< std::pair<OptAgent::ConstRef, OptAgent::ConstRef> >& agentPairs,
                   const planning2d::Time& timestamp,
                   ValueCollector& valueCollector,
                   double factor) {

  using namespace planning2d::time;
  for (auto& agentPair : agentPairs) {
    valueCollector.add(feature, feature.template evaluate<typename ValueCollector::ValueType>(timestamp, valueCollector.wrap(agentPair.first), valueCollector.wrap(agentPair.second)), factor, timestamp);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                            "Added error term at timestamp " << timestamp.format(Formatter(SEC,2)) << " and agent pair (" << agentPair.first.get().getId() << "," << agentPair.second.get().getId() << ")");
  }
}

template<typename FeatureType, typename ValueCollector>
ValueCollector& integrateSingletonFeature(const FeatureType& feature,
                               const ContinuousScene& scene,
                               ValueCollector& valueCollector) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: integrateSingletonFeature", false);

  // pre-computations
  const planning2d::Duration timestep(0.1); // TODO: where to get this from?
  const planning2d::Time tmin = scene.getMinTime();
  const planning2d::Time tmax = scene.getMaxTime();

  for (auto t=tmin; t<tmax; t+=timestep) {
    const auto& agents = feature.getSuitableAgents(t, scene);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                            "Got " << agents.size() << " suitable agents at timestamp " << t.format(Formatter(SEC,2)));
    sumOverAgents(feature, agents, t, valueCollector, timestep.toSec());  // iterate through all agents at current timestep
  }
  
  return valueCollector;
}

template<typename FeatureType, typename ValueCollector>
ValueCollector& integratePairwiseFeature(const FeatureType& feature,
                              const ContinuousScene& scene,
                              ValueCollector& valueCollector) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: integratePairwiseFeature", false);

  // pre-computations
  const planning2d::Duration timestep(0.1); // TODO: where to get this from?
  const planning2d::Time tmin = scene.getMinTime();
  const planning2d::Time tmax = scene.getMaxTime();

  for (auto t=tmin; t<tmax; t+=timestep) {
    const auto& agentPairs = feature.getSuitableAgentPairs(t, scene);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                            "Got " << agentPairs.size() << " suitable agent pairs at timestamp " << t.format(Formatter(SEC,2)));
    sumOverAgents(feature, agentPairs, t, valueCollector, timestep.toSec()); // iterate through all agents at current timestep
  }

  return valueCollector;
}

template<typename FeatureType, typename ValueCollector>
ValueCollector& sumOverObservationFeature(const FeatureType& feature,
                               const ContinuousScene& scene,
                               ValueCollector& valueCollector) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: sumOverObservationFeature", false);

  for (auto& snapshot : scene.getObservations()) {
    const planning2d::Time& stamp = snapshot->stamp();
    for (auto& meas : snapshot->objectContainer()) {
      if (scene.hasAgent(meas.first)) {
        auto& agent = scene.getOptAgent(meas.first);
        auto& trajectory = agent.trajectory();
        if (trajectory.contains(stamp)) {
          auto value = feature.template evaluate<typename ValueCollector::ValueType>(stamp, valueCollector.wrap(agent), meas.second);
          valueCollector.add(feature, value, 1.0, stamp);
          SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                                  "Added error term at timestamp " << stamp.format(Formatter(SEC,2)) << " for agent " << agent.getId() << " with value " << value);
        }
      }
    }
  }

  return valueCollector;
}

template<typename FeatureType, typename ValueCollector>
ValueCollector& applyTargetFeature(const FeatureType& feature,
                               const ContinuousScene& scene,
                               ValueCollector& valueCollector) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: applyTargetFeature", false);

  if (scene.hasAgent(ID_EGO)) {
    auto& agent = scene.getOptAgent(ID_EGO);   // TODO: so far target only known for robot, might want to switch to loop over agents
    auto& trajectory = agent.trajectory();
    auto stamp = trajectory.getFinalTime();
    auto value = feature.template evaluate<typename ValueCollector::ValueType>(stamp, valueCollector.wrap(agent));
    valueCollector.add(feature, value, 1.0, stamp);
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
        "Added error term at timestamp " << stamp.format(Formatter(SEC,2)) << " for agent " << agent.getId() << " with value " << value);
  } else {
    SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
        "Ego agent not present in scene");
  }

  return valueCollector;
}

template<typename FeatureType, typename ValueCollector>
ValueCollector& applyGridFeature(const FeatureType& feature,
                                 const ContinuousScene& scene,
                                 ValueCollector& valueCollector) {

  using namespace planning2d::time;
  Timer t("FeatureStatistics: applyGridFeature", false);

  // pre-computations
  const planning2d::Duration timestep(0.1); // TODO: where to get this from?
  const planning2d::Time tmin = scene.getMinTime();
  const planning2d::Time tmax = scene.getMaxTime();

  for (auto stamp=tmin; stamp<tmax; stamp+=timestep) {

    // Temporary: Just take the first grid
    // TODO: take the latest active grid
    SceneSnapshot::ConstPtr used_observation = nullptr;

    for (const auto& obs : scene.getObservations()) {
      if (obs->getOccupancyGrid()) {
        used_observation = obs;
        break;
      }
    }

    if (used_observation == nullptr)
      return valueCollector;

    // iterate through all agents at current timestep and add the error for this grid
    for (auto& agent : feature.getSuitableAgents(stamp, scene))
    {
      auto value = feature.template evaluate<typename ValueCollector::ValueType>(stamp, valueCollector.wrap(agent), *used_observation);
      valueCollector.add(feature, value, timestep.toSec(), stamp);
      SM_VERBOSE_STREAM_NAMED("feature_computation", __FUNCTION__ << "(" << feature.name() << "): "
                              "Added error term at timestamp " << stamp.format(planning2d::time::Formatter(planning2d::time::SEC,2)) << " for agent " << agent.get().getId());
    }
  }

  return valueCollector;
}

} /* namespace statistics */



/// \brief Important: Use this method in conjunction with auto return type from Eigen expressions!
template <typename T> // matches e.g. Eigen types
auto evalIfEigen(const T& m) -> decltype (m.eval()) {
  return m.eval();
}
template <int Rows, int Cols, typename NodeT> // matches GenericMatrixExpressions
auto evalIfEigen(const aslam::backend::GenericMatrixExpression<Rows, Cols, NodeT>& m) -> decltype(m) {
  return m;
}

template <typename T> // matches e.g. Eigen types and doubles
inline auto evaluateIfExpression(const T& m) -> decltype(m) {
  return m;
}
template <typename T = aslam::backend::ScalarExpression> // matches ScalarExpressions
inline auto evaluateIfExpression(const aslam::backend::ScalarExpression& m) -> decltype (m.evaluate()) {
  return m.evaluate();
}
template <int Rows, int Cols, typename NodeT> // matches GenericMatrixExpressions
inline auto evaluateIfExpression(const aslam::backend::GenericMatrixExpression<Rows, Cols, NodeT>& m) -> decltype (m.evaluate()) {
  return m.evaluate();
}



namespace collectors {

  class TrajectoryExpressionWrapper {
   public:
    TrajectoryExpressionWrapper(const Trajectory & t) : _t(t) {}

    Trajectory::TrajectoryValueExpression getPosition(const planning2d::Time& timestamp) const {
      return _t.getExpressionFromSplinePos2d(timestamp);
    }

    Trajectory::TrajectoryValueExpression getVelocityXY(const planning2d::Time& timestamp) const {
      return _t.getExpressionFromSplineVelXY(timestamp);
    }

    Trajectory::TrajectoryValueExpression getAccelerationXY(const planning2d::Time& timestamp) const {
      return _t.getExpressionFromSplineAccXY(timestamp);
    }

    aslam::backend::GenericMatrixExpression<1, 1> getRotationRateSquared(const planning2d::Time& timestamp) const {
      return _t.getExpressionFromSplineRotationRateSquared(timestamp);
    }

    planning2d::Time getFinalTime() const {
      return _t.getFinalTime();
    }

    planning2d::Time getStartTime() const {
      return _t.getStartTime();
    }

    //TODO add more delegating functions, as needed
   private:
    const Trajectory & _t;
  };

  class OptAgentExpressionWrapper : public OptAgent {
    public:
      OptAgentExpressionWrapper(const OptAgent & oa) : OptAgent(oa) { }

      TrajectoryExpressionWrapper trajectory() const { return TrajectoryExpressionWrapper(OptAgent::trajectory()); }
  };

  template <int Dim>
  struct ReturnTypeTraits {
    typedef Eigen::Matrix<double, Dim, 1> ValueType;
    typedef aslam::backend::GenericMatrixExpression<Dim, 1> ExpressionType;
    static Eigen::VectorXd toEigenVectorXd(const ValueType& v) { return v; }
    static ValueType Zero() { return ValueType::Zero(); }
  };

  template <>
  struct ReturnTypeTraits<1> {
    typedef double ValueType;
    typedef aslam::backend::GenericMatrixExpression<1, 1> ExpressionType;
    static Eigen::VectorXd toEigenVectorXd(const ValueType& v) { return (Eigen::VectorXd(1, 1) << v).finished(); }
    static ValueType Zero() { return 0.0; }
  };


  template <int Dim>
  class OptimizationProblemValueCollector {
   public:
    typedef typename ReturnTypeTraits<Dim>::ExpressionType ValueType;
    OptimizationProblemValueCollector(aslam::backend::OptimizationProblem &op) : _op(op) { }

    void add(const RawFeature & f, aslam::backend::GenericMatrixExpression<Dim, 1> exp, double factor, const planning2d::Time& stamp) {
      _op.addErrorTerm( boost::make_shared<RawFeatureErrorTerm<Dim> >(f, exp, factor, stamp));
    }

    OptAgentExpressionWrapper wrap(const OptAgent & oa) {
      return oa;
    }

   private:
    aslam::backend::OptimizationProblem &_op;
  };

  template <int Dim>
  class ValueAdder {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef typename ReturnTypeTraits<Dim>::ValueType ValueType;
    /// \brief Default constructor initializes return value to zero.
    /// Important when no values were added since a feature cannot be computed on a given scene.
    /// In this case, both the feature expectation and the feature demonstration should be zero, hence the gradient also zero.
    ValueAdder() : _val(ReturnTypeTraits<Dim>::Zero()) { }
    template<typename Derived>
    void add(const RawFeature & /*f*/, const Eigen::MatrixBase<Derived> & v, double factor, const planning2d::Time& /*stamp*/) {
      _val +=  v * factor;
    }

    void add(const RawFeature & /*f*/, const double v, double factor, const planning2d::Time& /*stamp*/) {
      _val += v * factor;
    }

    Eigen::VectorXd getValue() {
      return ReturnTypeTraits<Dim>::toEigenVectorXd(_val);
    }

    operator ValueType & () {
      return getValue();
    }

    const OptAgent & wrap(const OptAgent & oa) {
      return oa;
    }

   private:
    ValueType _val;
  };
}

template <typename Derived, int Dim = 1>
class IntegratedSingletonFeature : public RawFeatureSingleton {
 public:
  IntegratedSingletonFeature(const std::string& name,
                             const OptAgentType& optAgentType,
                             const double scalingFactor = 1.0,
                             const bool activateScaling = true) : RawFeatureSingleton(name, optAgentType, Dim, scalingFactor, activateScaling) { }
  IntegratedSingletonFeature(const std::string& name,
                             const sm::value_store::ValueStore& vpt) : RawFeatureSingleton(name, vpt, Dim) { }


  Eigen::VectorXd evaluate(const ContinuousScene & scene) const override {
    collectors::ValueAdder<Dim> va;
    return statistics::integrateSingletonFeature(getDerived(), scene, va).getValue().cwiseProduct(getScalingFactor());
  }

  void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const override {
    collectors::OptimizationProblemValueCollector<Dim> oc(optimizationProblem);
    statistics::integrateSingletonFeature(getDerived(), scene, oc);
  }

  aslam::backend::GenericMatrixExpression<1, 1> getExpression(const planning2d::Time& timestamp, const OptAgent& agent) const override {
    return getDerived().template evaluate<typename collectors::OptimizationProblemValueCollector<Dim>::ValueType>(timestamp, collectors::OptAgentExpressionWrapper(agent));
  }

 protected:
  IntegratedSingletonFeature() : RawFeatureSingleton() { }

 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }
};

template <typename Derived, int Dim = 1>
class IntegratedPairwiseFeature : public RawFeaturePairwise {
 public:
  IntegratedPairwiseFeature(const std::string& name,
                            const OptAgentType& optAgentType,
                            const double scalingFactor = 1.0,
                            const bool activateScaling = true) : RawFeaturePairwise(name, optAgentType, Dim, scalingFactor, activateScaling) { }
  IntegratedPairwiseFeature(const std::string& name,
                            const sm::value_store::ValueStore& vpt) : RawFeaturePairwise(name, vpt, Dim) { }

  Eigen::VectorXd evaluate(const ContinuousScene & scene) const override {
    collectors::ValueAdder<Dim> va;
    return statistics::integratePairwiseFeature(getDerived(), scene, va).getValue().cwiseProduct(getScalingFactor());
  }

  void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const override {
    collectors::OptimizationProblemValueCollector<Dim> oc(optimizationProblem);
    statistics::integratePairwiseFeature(getDerived(), scene, oc);
  }

 protected:
  IntegratedPairwiseFeature() : RawFeaturePairwise() { }

 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }
};

template <typename Derived, int Dim = 1>
class ObservationSingletonFeature : public RawFeature {
 public:
  ObservationSingletonFeature(const std::string& name,
                            const OptAgentType& optAgentType,
                            const double scalingFactor = 1.0,
                            const bool activateScaling = true) : RawFeature(name, optAgentType, Dim, scalingFactor, activateScaling) { }
  ObservationSingletonFeature(const std::string& name,
                            const sm::value_store::ValueStore& vpt) : RawFeature(name, vpt, Dim) { }

  Eigen::VectorXd evaluate(const ContinuousScene & scene) const override {
    collectors::ValueAdder<Dim> va;
    return statistics::sumOverObservationFeature(getDerived(), scene, va).getValue().cwiseProduct(getScalingFactor());
  }

  void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const override {
    collectors::OptimizationProblemValueCollector<Dim> oc(optimizationProblem);
    statistics::sumOverObservationFeature(getDerived(), scene, oc);
  }

 protected:
  ObservationSingletonFeature() : RawFeature() { }

 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }

};

template <typename Derived, int Dim = 1>
class TargetSingletonFeature : public RawFeature {
 public:
  TargetSingletonFeature(const std::string& name,
                         const OptAgentType& optAgentType,
                         const planning2d::Pose2d& target,
                         const double scalingFactor = 1.0,
                         const bool activateScaling = true) : RawFeature(name, optAgentType, Dim, scalingFactor, activateScaling), _target(target) { }
  TargetSingletonFeature(const std::string& name,
                         const sm::value_store::ValueStore& vpt,
                         const planning2d::Pose2d& target) : RawFeature(name, vpt, Dim), _target(target)  { }

  Eigen::VectorXd evaluate(const ContinuousScene & scene) const override {
    collectors::ValueAdder<Dim> va;
    return statistics::applyTargetFeature(getDerived(), scene, va).getValue().cwiseProduct(getScalingFactor());
  }

  void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const override {
    collectors::OptimizationProblemValueCollector<Dim> oc(optimizationProblem);
    statistics::applyTargetFeature(getDerived(), scene, oc);
  }

  const planning2d::Pose2d& getTarget() const { return _target; }
  void setTarget(const planning2d::Pose2d& target) { _target = target; }

 protected:
  TargetSingletonFeature() : RawFeature() { }

 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }

  planning2d::Pose2d _target;

};

template <typename Derived, int Dim = 1>
class GridSingletonFeature : public RawFeatureSingleton {
 public:
  GridSingletonFeature(const std::string& name, const OptAgentType& optAgentType, const double scalingFactor = 1.0, const bool activateScaling = true)
      : RawFeatureSingleton(name, optAgentType, Dim, scalingFactor, activateScaling) { }
  GridSingletonFeature(const std::string& name, const sm::value_store::ValueStore& vpt)
      : RawFeatureSingleton(name, vpt, Dim) { }

  Eigen::VectorXd evaluate(const ContinuousScene & scene) const override {
    collectors::ValueAdder<Dim> va;
    return statistics::applyGridFeature(getDerived(), scene, va).getValue().cwiseProduct(getScalingFactor());
  }

  void addErrorTerms(const ContinuousScene& scene, OptimizationProblem& optimizationProblem) const override {
    collectors::OptimizationProblemValueCollector<Dim> oc(optimizationProblem);
    statistics::applyGridFeature(getDerived(), scene, oc);
  }

 protected:
  GridSingletonFeature() : RawFeatureSingleton() { }

 private:
  const Derived & getDerived() const {
    return static_cast<const Derived &>(*this);
  }

};


} /* namespace prob_planner */


#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESTATISTICS_HPP_ */
