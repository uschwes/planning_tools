/*
 * ProbabilisticLearnerPy.cpp
 *
 *  Created on: 20.08.2015
 *      Author: Ulrich Schwesinger
 */

//#include <sys/signal.h>
//#include <cstdlib>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces_python/SupportPy.hpp>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/ProbabilisticLearner.hpp>

#include <probabilistic_planner_python/FeatureContainerModifiable.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>
//using namespace std;
using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

//class RunKillable {
//public:
//  RunKillable(ProbabilisticLearner& learner) : _signals(_io_service) {
//    _signals.add(SIGINT);
//    _signals.add(SIGTERM);
//    _signals.async_wait(boost::bind(&RunKillable::stop, this));
//    _thread = boost::thread(boost::bind(&boost::asio::io_service::run, &_io_service));
//    _io_service.post(boost::bind(&ProbabilisticLearner::run, learner));
//    _thread.join();
//  }
//  void stop() {
//    SM_INFO_STREAM("CTRL+C pressed, killing ProbabilisticLearner::run() thread!");
//    _io_service.stop();
//    _thread.interrupt();
//  }
//private:
//  boost::asio::io_service _io_service;
//  boost::asio::signal_set _signals;
//  boost::thread _thread;
//};
//
//void runKillable(ProbabilisticLearner& learner) {
//  RunKillable r(learner);
//}

//ProbabilisticLearner::Ptr constructorWrapper(FeatureContainer& features,
//                                             const boost::python::list& lScenes,
//                                             const OptimizerRpropOptions& options = OptimizerRpropOptions(),
//                                             const ErrorTermLearning::Options& etOptions = ErrorTermLearning::Options()) {
//  vector<ContinuousScene::Ptr> scenes;
//  scenes.reserve(boost::python::len(lScenes));
//  for (int i=0; i<boost::python::len(lScenes); i++)
//    scenes.push_back(boost::python::extract<ContinuousScene::Ptr>(lScenes[i]));
//  ProbabilisticLearner::Ptr learner(new ProbabilisticLearner(features, scenes, options, etOptions));
//  return learner;
//}

//BOOST_PYTHON_FUNCTION_OVERLOADS(ProbabilisticLearner_constructorWrapper_overloads, constructorWrapper, 2, 4);

boost::python::list getSamplesWrapper(const ProbabilisticLearner& learner) {
  boost::python::list l;
  for (size_t i=0; i<learner.nSamplers(); i++)
    l.append(learner.getSamples(i));
  return l;
}

boost::python::list getSamplersWrapper(const ProbabilisticLearner& learner) {
  boost::python::list l;
  for (const auto& sampler : learner.getSamplers())
    l.append(sampler);
  return l;
}

boost::python::list getFeatureInfoWrapper(const ProbabilisticLearner& learner) {
  boost::python::list ll;
  for (size_t i=0; i<learner.nSamplers(); i++) {
    const auto& fiv = learner.getFeatureInfo(i);
    boost::python::list l;
    for (const auto& fi : fiv)
      l.append(fi);
    ll.append(l);
  }
  return ll;
}

std::string printFeatureInfoWrapper(const boost::python::list& ll) {
  std::vector<ErrorTermLearning::FeatureInfo> featureInfo;
  for (size_t i=0; i<static_cast<std::size_t>(boost::python::len(ll)); i++) {
    boost::python::extract<const ErrorTermLearning::FeatureInfo&> extractor(ll[i]);
    SM_ASSERT_TRUE(planning2d::RuntimeException, extractor.check(), "Wrong type provided");
    featureInfo.push_back(extractor());
  }
  std::ostringstream os;
  os << featureInfo;
  return os.str();
}

#define WRAP_EIGEN_MEMBER(CLASS_NAME, MEMBER_NAME, FCN_NAME) \
Eigen::VectorXd wrap##FCN_NAME(const CLASS_NAME& f) { \
  return f.MEMBER_NAME; \
}

WRAP_EIGEN_MEMBER(ErrorTermLearning::FeatureInfo, weights, Weights);
WRAP_EIGEN_MEMBER(ErrorTermLearning::FeatureInfo, demonstration, Demonstration);
WRAP_EIGEN_MEMBER(ErrorTermLearning::FeatureInfo, expectation, Expectation);

void exportProbabilisticLearner() {

  class_<ErrorTermLearning::FeatureInfo>("ErrorTermLearningFeatureInfo", init<>("ErrorTermLearningFeatureInfo(): Default constructor"))
    .add_property("name", &ErrorTermLearning::FeatureInfo::name)
    .add_property("weights", &wrapWeights)
    .add_property("demonstration", &wrapDemonstration)
    .add_property("expectation", &wrapExpectation)
    .add_property("mode", boost::python::detail::make_function_aux([](ErrorTermLearning::FeatureInfo& fi){ return fi.mode;}, return_value_policy<return_optional>(),
                          boost::mpl::vector<boost::optional<Eigen::VectorXd>,ErrorTermLearning::FeatureInfo>()))
    .def_readonly("numActiveDesignVariables", &ErrorTermLearning::FeatureInfo::numActiveDesignVariables)
    .def("__str__", &printFeatureInfoWrapper)
    .def_pickle(BoostSerializationBinary_pickle_suite<ErrorTermLearning::FeatureInfo>())
  ;

  class_<ErrorTermLearning::Options>("ErrorTermLearningOptions", init<>("ErrorTermLearningOptions(): Default constructor"))
    .def_readwrite("nMcmcStepsBurnIn", &ErrorTermLearning::Options::nMcmcStepsBurnIn)
    .def_readwrite("nMcmcSamplesForMean", &ErrorTermLearning::Options::nMcmcSamplesForMean)
    .def_readwrite("nThin", &ErrorTermLearning::Options::nThin)
    .def_readwrite("storeSamples", &ErrorTermLearning::Options::storeSamples)
    .def_readwrite("initializeAtMode", &ErrorTermLearning::Options::initializeAtMode)
    .def_readwrite("resetToDemonstration", &ErrorTermLearning::Options::resetToDemonstration)
  ;

  class_<ErrorTermLearning, ErrorTermLearning::Ptr, bases<aslam::backend::ScalarNonSquaredErrorTerm>, boost::noncopyable>("ErrorTermLearning", no_init)
    .add_property("options", make_function(&ErrorTermLearning::getOptions, return_value_policy<copy_const_reference>()), &ErrorTermLearning::setOptions)
    .add_property("effectiveSampleSize", &ErrorTermLearning::getEffectiveSampleSize)
    .add_property("integratedAutocorrelationTime", &ErrorTermLearning::getIntegratedAutocorrelationTime)
    .add_property("mode", &ErrorTermLearning::getMode)
  ;
  register_ptr_to_python< ErrorTermLearning::ConstPtr >();
  implicitly_convertible<ErrorTermLearning::Ptr, ErrorTermLearning::ConstPtr >();

  enum_<ProbabilisticLearner::Options::SamplerType>("SamplerType")
    .value("METROPOLIS_HASTINGS", ProbabilisticLearner::Options::METROPOLIS_HASTINGS)
    .value("HYBRID_MONTE_CARLO", ProbabilisticLearner::Options::HYBRID_MONTE_CARLO)
  ;

  class_<ProbabilisticLearner::Options>("ProbabilisticLearnerOptions", init<>("ProbabilisticLearnerOptions(): Default constructor"))
    .def_readwrite("rpropOptions", &ProbabilisticLearner::Options::rpropOptions)
    .def_readwrite("etOptions", &ProbabilisticLearner::Options::etOptions)
    .def_readwrite("hybridMonteCarloOptions", &ProbabilisticLearner::Options::hybridMonteCarloOptions)
    .def_readwrite("metropolisHastingsOptions", &ProbabilisticLearner::Options::metropolisHastingsOptions)
    .def_readwrite("samplerType", &ProbabilisticLearner::Options::samplerType)
  ;

  class_<ProbabilisticLearner, ProbabilisticLearner::Ptr>("ProbabilisticLearner", no_init)
    .def(init<FeatureContainer&, optional<const ProbabilisticLearner::Options&> >
      ("ProbabilisticLearner(FeatureContainer, ContinuousSceneList, ProbabilisticLearnerOptions[optional])"))
//    .def("__init__", &constructorWrapper, ProbabilisticLearner_constructorWrapper_overloads())
    .def("run", &ProbabilisticLearner::run)
    .def("addDemonstrations", &ProbabilisticLearner::addDemonstrations)
    .add_property("nSamplers", &ProbabilisticLearner::nSamplers)
    .add_property("samples", &getSamplesWrapper)
    .def("getErrorTerm", &ProbabilisticLearner::getErrorTerm)
    .def("getEffectiveSampleSize", &ProbabilisticLearner::getEffectiveSampleSize)
    .def("getIntegratedAutocorrelationTime", &ProbabilisticLearner::getIntegratedAutocorrelationTime)
    .add_property("samplers", &getSamplersWrapper)
    .add_property("featureInfo", &getFeatureInfoWrapper)
    .add_property("optimizer", make_function(&ProbabilisticLearner::optimizer, return_value_policy<copy_const_reference>()))
    .add_property("options", make_function(&ProbabilisticLearner::getOptions, return_internal_reference<>()))
    .def("setErrorTermOptions", &ProbabilisticLearner::setErrorTermOptions)
    .def("setOptimizerRpropOptions", &ProbabilisticLearner::setOptimizerRpropOptions)
  ;

}

