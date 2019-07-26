#include <numpy_eigen/boost_python_headers.hpp>
#include <probabilistic_planner/Support.hpp>

using namespace std;
using namespace boost::python;

void exportOptAgent();
void exportTrajectory();
void exportContinuousScene();
void exportSceneSnapshot();
void exportFeatureContainer();
void exportRawFeature();
void exportRawFeatureSingleton();
void exportRawFeaturePairwise();
void exportFeatures();
void exportFeatureScaler();
void exportProbabilisticLearner();
void exportAddGridErrorTerm2d();
void exportAutocorrelation();
void exportDensityComputations();

BOOST_PYTHON_MODULE(libprobabilistic_planner_python) {
  exportOptAgent();
  exportTrajectory();
  exportContinuousScene();
  exportSceneSnapshot();
  exportFeatureContainer();
  exportRawFeature();
  exportRawFeatureSingleton();
  exportRawFeaturePairwise();
  exportFeatures();
  exportFeatureScaler();
  exportProbabilisticLearner();
  exportAddGridErrorTerm2d();
  exportAutocorrelation();
  exportDensityComputations();

  boost::python::scope().attr("ID_EGO") = prob_planner::ID_EGO;
}
