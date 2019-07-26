/*
 * FeatureImplementationsPy.cpp
 *
 *  Created on: Jul 16, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

// Pairwise features
#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedSigmoidDistance.hpp>

// Physical trajectory attribute features
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp>

// Observation features
#include <probabilistic_planner/features/FeatureSingletonObservationAbsoluteVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationHeading.hpp>

// Barrier functions / soft constraints
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>

// Target features
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>

// Grid features
#include <probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp>

// Product features
#include <probabilistic_planner/features/FeatureProduct.hpp>

using namespace boost::python;
using namespace prob_planner;

#define ADD_ERROR_TERMS(FEATURE_TYPE) "addErrorTerms", &FEATURE_TYPE::addErrorTerms, "Add error terms to optimization problem"
#define EVALUATE(FEATURE_TYPE) "evaluate", (Eigen::VectorXd (FEATURE_TYPE::*) (const ContinuousScene &) const) &FEATURE_TYPE::evaluate, "Compute value of feature (vector for multiple dimensions)"
#define IMPLICITLY_CONVERTIBLY(FEATURE_TYPE) implicitly_convertible<typename FEATURE_TYPE::Ptr,typename FEATURE_TYPE::ConstPtr >()
#define FEATURE_CONSTRUCTOR_DEFAULT(FEATURE_NAME) init<const OptAgentType&, const double, optional<double, bool> >( #FEATURE_NAME "(type, weight, double scalingFactor = 1.0, bool activateScaling = true): constructor")
#define VALUE_STORE_FEATURE_CONSTRUCTOR(FEATURE_NAME) init<const sm::value_store::ValueStore&>( #FEATURE_NAME "(ValueStore vs): constructor")

template <typename Feature, typename Base>
void exportFeature(const std::string& name) {
  class_<Feature, bases<Base> >(name.c_str(), FEATURE_CONSTRUCTOR_DEFAULT(name))
    .def(VALUE_STORE_FEATURE_CONSTRUCTOR(Feature))
    .def(ADD_ERROR_TERMS(Feature))
    .def(EVALUATE(Feature))
  ;

  implicitly_convertible<typename Feature::Ptr,typename Feature::ConstPtr >();
}


void exportFeatures() {

  exportFeature<FeaturePairwiseIntegratedDistance, RawFeaturePairwise>("FeaturePairwiseIntegratedDistance");
  exportFeature<FeaturePairwiseIntegratedInverseDistance, RawFeaturePairwise>("FeaturePairwiseIntegratedInverseDistance");
  exportFeature<FeatureSingletonIntegratedAcceleration, RawFeatureSingleton>("FeatureSingletonIntegratedAcceleration");
  exportFeature<FeatureSingletonIntegratedRotationRate, RawFeatureSingleton>("FeatureSingletonIntegratedRotationRate");
  exportFeature<FeatureSingletonIntegratedVelocity, RawFeatureSingleton>("FeatureSingletonIntegratedVelocity");
  exportFeature<FeatureSingletonIntegratedDirectionOfMotion, RawFeatureSingleton>("FeatureSingletonIntegratedDirectionOfMotion");
  exportFeature<FeatureSingletonObservationAbsoluteVelocity, RawFeature>("FeatureSingletonObservationAbsoluteVelocity");
  exportFeature<FeatureSingletonObservationVelocityVector, RawFeature>("FeatureSingletonObservationVelocityVector");
  exportFeature<FeatureSingletonObservationPosition2d, RawFeature>("FeatureSingletonObservationPosition2d");
  exportFeature<FeatureSingletonObservationHeading, RawFeature>("FeatureSingletonObservationHeading");

  // Manually export features with more functions

  class_<FeatureSingletonIntegratedBarrierVelocity, bases<RawFeatureSingleton> >("FeatureSingletonIntegratedBarrierVelocity",
       init<const OptAgentType&, const double, optional<double>>("FeatureSingletonIntegratedBarrierVelocity(type, weight, maximumVelocity[optional]): constructor"))
    .def(VALUE_STORE_FEATURE_CONSTRUCTOR(FeatureSingletonIntegratedBarrierVelocity))
    .def(ADD_ERROR_TERMS(FeatureSingletonIntegratedBarrierVelocity))
    .def(EVALUATE(FeatureSingletonIntegratedBarrierVelocity))
    .add_property("maximumVelocity", &FeatureSingletonIntegratedBarrierVelocity::getMaximumVelocity, &FeatureSingletonIntegratedBarrierVelocity::setMaximumVelocity, "maximum velocity")
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonIntegratedBarrierVelocity);

  class_<FeatureSingletonIntegratedVelocityDifference, bases<RawFeatureSingleton> >("FeatureSingletonIntegratedVelocityDifference",
       init<const OptAgentType&, const double, optional<double>>("FeatureSingletonIntegratedVelocityDifference(type, weight, desiredVelocity[optional]): constructor"))
    .def(VALUE_STORE_FEATURE_CONSTRUCTOR(FeatureSingletonIntegratedBarrierVelocity))
    .def(ADD_ERROR_TERMS(FeatureSingletonIntegratedVelocityDifference))
    .def(EVALUATE(FeatureSingletonIntegratedVelocityDifference))
    .add_property("desiredVelocity", &FeatureSingletonIntegratedVelocityDifference::getDesiredVelocity, &FeatureSingletonIntegratedVelocityDifference::setDesiredVelocity, "desired velocity")
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonIntegratedVelocityDifference);

  class_<FeatureSingletonRobotTarget, bases<RawFeatureSingleton> >("FeatureSingletonRobotTarget",
       init<const OptAgentType&, const double, optional<const planning2d::Pose2d&>>("FeatureSingletonRobotTarget(type, weight, target[optional]): constructor"))
    .def(ADD_ERROR_TERMS(FeatureSingletonRobotTarget))
    .def(EVALUATE(FeatureSingletonRobotTarget))
    .add_property("target", make_function(&FeatureSingletonRobotTarget::getTarget, return_internal_reference<>()), &FeatureSingletonRobotTarget::setTarget, "robot target")
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonRobotTarget);

  class_<FeatureSingletonIntegratedStaticObstacleDistance, bases<RawFeatureSingleton> >("FeatureSingletonIntegratedStaticObstacleDistance", FEATURE_CONSTRUCTOR_DEFAULT(FeatureSingletonIntegratedStaticObstacleDistance))
    .def(ADD_ERROR_TERMS(FeatureSingletonIntegratedStaticObstacleDistance))
    .def(EVALUATE(FeatureSingletonIntegratedStaticObstacleDistance))
    .def("setSigmoidParameters", &FeatureSingletonIntegratedStaticObstacleDistance::setSigmoidParameters)
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonIntegratedStaticObstacleDistance);

  class_<FeaturePairwiseIntegratedSigmoidDistance, bases<RawFeaturePairwise> >("FeaturePairwiseIntegratedSigmoidDistance", FEATURE_CONSTRUCTOR_DEFAULT(FeaturePairwiseIntegratedSigmoidDistance))
    .def(ADD_ERROR_TERMS(FeaturePairwiseIntegratedSigmoidDistance))
    .def(EVALUATE(FeaturePairwiseIntegratedSigmoidDistance))
    .def("setSigmoidParameters", &FeaturePairwiseIntegratedSigmoidDistance::setSigmoidParameters)
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonIntegratedStaticObstacleDistance);

  class_<FeatureSingletonProduct, bases<RawFeatureSingleton> >("FeatureSingletonProduct", no_init)
    .def(init<const RawFeatureSingleton::ConstPtr&, const RawFeatureSingleton::ConstPtr&, const OptAgentType&, const double, optional<double, bool> >(
        "FeatureSingletonProduct(FeatureSingleton lhs, FeatureSingleton rhs, type, weight, double scalingFactor = 1.0, bool activateScaling = true): constructor"))
    .def(init<const sm::value_store::ValueStore&, const FeatureContainer&>("FeatureSingletonProduct(ValueStore vs, FeatureContainer container): constructor"))
    .def(ADD_ERROR_TERMS(FeatureSingletonProduct))
//    .def((Eigen::VectorXd (FeatureSingletonProduct::*) (const ContinuousScene &) const) &FeatureSingletonProduct::evaluate)
  ;
  IMPLICITLY_CONVERTIBLY(FeatureSingletonProduct);


} /* void exportFeatures() */
