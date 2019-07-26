#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/GenericScalarExpression.hpp>
#include <aslam/backend/util/ExpressionUtils.hpp>

#include <planner_interfaces/Support.hpp>

#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/Support.hpp>

namespace prob_planner {

  template <int Dim>
  class RawFeatureErrorTerm : public aslam::backend::ScalarNonSquaredErrorTerm {

   public:
    PLANNING_2D_POINTER_TYPEDEFS(RawFeatureErrorTerm);

   public:
    RawFeatureErrorTerm(const RawFeature & f, aslam::backend::GenericMatrixExpression<Dim, 1> exp, double factor, const planning2d::Time& stamp) : _f(f), _exp(exp) {
      assert(f.numWeights() == Dim);
      setWeight(factor);
      setTime(stamp.nanosec);
      aslam::backend::DesignVariable::set_t dvs;
      exp.getDesignVariables(dvs);
      setDesignVariablesIterator(dvs.begin(), dvs.end());
    }

    const RawFeature& feature() const { return _f; }

   private:
    /// \brief Evaluate the error (negative log density)
    inline double evaluateErrorImplementation() override {
      ::prob_planner::Timer timer(_f.name() + ": " + __FUNCTION__, false);
      auto vec = _exp.evaluate();
      SM_ASSERT_TRUE(planning2d::RuntimeException, vec.allFinite(),
                     "Error " << vec.transpose() << " for feature " << _f.name() << 
                     " (factor " << getWeight() << ") has elements which are not finite.");
      return _f.getCurrentWeightsVector().transpose() * _f.getScalingFactor().cwiseProduct(vec);
    }

    inline void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) override {
      ::prob_planner::Timer timer(_f.name() + ": " + __FUNCTION__, false);
      try {
        _exp.evaluateJacobians(outJacobians.apply(_f.getCurrentWeightsVector().transpose().cwiseProduct(_f.getScalingFactor())));
      } catch (std::exception& e) {
        SM_ERROR_STREAM("Exception in " << __FUNCTION__ << "(" << _f.name() << "): " << e.what());
        throw;
      }
      SM_ASSERT_TRUE(planning2d::RuntimeException, aslam::backend::utils::isFinite(outJacobians, _exp),
                     "Gradient for feature " << _f.name() << " (factor " << getWeight() << ") "
                     "has elements which are not finite." << std::endl << outJacobians.asDenseMatrix());
    }

    const RawFeature & _f;
    aslam::backend::GenericMatrixExpression<Dim, 1> _exp;
  };

} /* namespace prob_planner */
