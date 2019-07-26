/*
 * FeatureWeightScalarDesignVariable.hpp
 *
 *  Created on: 02.10.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREWEIGHTSCALARDESIGNVARIABLE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREWEIGHTSCALARDESIGNVARIABLE_HPP_

#include <algorithm>    // std::max
#include <aslam/backend/Scalar.hpp>
#include <planner_interfaces/Support.hpp>

namespace prob_planner {

  class FeatureWeightScalarDesignVariable : public aslam::backend::Scalar {

   public:
    PLANNING_2D_POINTER_TYPEDEFS(FeatureWeightScalarDesignVariable);

   public:
    /// \brief Constructor
    inline FeatureWeightScalarDesignVariable(const double & p);
    /// \brief Constructor
    inline FeatureWeightScalarDesignVariable(const double & p, bool forbidNegative);

    /// \brief Update the design variable.
    inline virtual void updateImplementation(const double * dp, int size) override;

    /// \brief Activate/deactivate clamping at zero
    void forbidNegative(bool f) { _forbidNegative = f; }

   private:

    /// \brief Assures the design variable is non-negative
    inline void clampAtZero();

   private:
    bool _forbidNegative = true; /// \brief Will clamp the feature weight at zero if set to true
  };



  FeatureWeightScalarDesignVariable::FeatureWeightScalarDesignVariable(const double & p) :
    Scalar(p) {
    clampAtZero();
  }

  FeatureWeightScalarDesignVariable::FeatureWeightScalarDesignVariable(const double & p, bool forbidNegative) :
      Scalar(p),
      _forbidNegative(forbidNegative) {
    clampAtZero();
  }

  void FeatureWeightScalarDesignVariable::updateImplementation(const double * dp, int /*size*/) {
    Scalar::updateImplementation(dp, 1);
    clampAtZero();
  }

  void FeatureWeightScalarDesignVariable::clampAtZero() {
    if (_forbidNegative && toScalar() < 0.0)
      setParameters(Eigen::MatrixXd::Zero(1,1));
  }

}

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREWEIGHTSCALARDESIGNVARIABLE_HPP_ */
