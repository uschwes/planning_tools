/*
 * ContinousScenePy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

#include <probabilistic_planner/features/CacheEntryDistanceTransform.hpp>
#include <common_agents/HolonomicAgent.hpp>
#include <common_agents/DifferentialDriveAgent.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

typedef vector<ContinuousScene::Ptr> ContinuousSceneList;

boost::python::dict getOptAgents(const ContinuousScene& scene) {

  boost::python::dict dct;
  for (auto& idAgentPair : scene.getOptAgentContainer()) {
    dct[idAgentPair.first] = idAgentPair.second;
  }
  return dct;
}

boost::python::list getObservationsWrapper(const ContinuousScene& scene) {
  boost::python::list observations;
  for (auto& obs : scene.getObservations()) {
    observations.append(obs);
  }
  return observations;
}

ContinuousScene::Ptr copyWrapper(const ContinuousScene& scene) {
  ContinuousScene::Ptr c(new ContinuousScene());
  scene.copy(*c);
  return c;
}

class ContinuousSceneModifiable : public ContinuousScene {
 public:
  PLANNING_2D_POINTER_TYPEDEFS(ContinuousSceneModifiable);
 public:
  ContinuousSceneModifiable() : ContinuousScene() {}
  ContinuousSceneModifiable(const ContinuousScene& scene) {
    scene.copy(*this);
  }
  using ContinuousScene::setActiveSplineParameters;
};

template <typename Archive>
struct ContinuousSceneTypeRegistry {
  static void registerTypes(Archive& a) {
    a.template register_type<CacheEntryDistanceTransform>();
    a.template register_type<common_agents::HolonomicSystemInput>();
    a.template register_type<common_agents::HolonomicSystemInputStamped>();
    a.template register_type<common_agents::HolonomicState>();
    a.template register_type<common_agents::HolonomicStateStamped>();
    a.template register_type<common_agents::HolonomicAgent>();
    a.template register_type<common_agents::DifferentialDriveSystemInput>();
    a.template register_type<common_agents::DifferentialDriveSystemInputStamped>();
    a.template register_type<common_agents::DifferentialDriveState>();
    a.template register_type<common_agents::DifferentialDriveStateStamped>();
    a.template register_type<common_agents::DifferentialDriveAgent>();
  }
};

template <typename CS>
struct ContinuousScene_pickle_suite : public BoostSerializationBinary_pickle_suite<CS,ContinuousSceneTypeRegistry> {
  typedef BoostSerializationBinary_pickle_suite<CS,ContinuousSceneTypeRegistry> parent_t;
  virtual ~ContinuousScene_pickle_suite() { }
};

void serializeScene(const ContinuousScene& scene, const string& filename) {
  std::ofstream of(filename);
  boost::archive::binary_oarchive oa(of);
  ContinuousSceneTypeRegistry<boost::archive::binary_oarchive>::registerTypes(oa);
  oa << scene;
}

template <typename T>
T getSlice(const T& self, const boost::python::slice& slice)
{
  T result;

  boost::python::slice::range<typename T::const_iterator> range;
  try {
    range = slice.get_indices(self.begin(), self.end());
  } catch (std::invalid_argument& e) {
    return result;
  }

  for (; range.start != range.stop; std::advance(range.start, range.step)) {
    result.push_back(*range.start);
  }
  result.push_back(*range.start); // Handle last item.

  return result;
}

void exportContinuousScene() {

  class_<vector<SceneSnapshot> >("SnapshotContainer")
    .def(boost::python::vector_indexing_suite<vector<SceneSnapshot> >())
  ;

  class_<ContinuousSceneList>("ContinuousSceneList")
    .def(boost::python::vector_indexing_suite<ContinuousSceneList, true>())
    .def("__init__", make_constructor(&vector_shared_ptr_from_list<ContinuousScene::Ptr>))
    .def("__init__", make_constructor(&vector_shared_ptr_from_object<ContinuousScene::Ptr>))
    .def("__getitem__", &getSlice<ContinuousSceneList>)
    .def_pickle(ContinuousScene_pickle_suite<ContinuousSceneList>())
  ;

  class_<ContinuousScene, ContinuousScene::Ptr, boost::noncopyable>("ContinuousScene", init<>("ContinuousScene(): Default constructor"))
    .def("clear", &ContinuousScene::clear)
    .def("copy", &copyWrapper)
    .def("empty", &ContinuousScene::empty)
    .def("numberOfAgents", &ContinuousScene::numberOfAgents)
    .add_property("optAgents", &getOptAgents, &ContinuousScene::setOptAgentContainer)
    .def("hasAgent", &ContinuousScene::hasAgent)
    .def("addOptAgent", &ContinuousScene::addOptAgent)
    .def("updateOptAgent", &ContinuousScene::updateOptAgent)
    .def("getOptAgent", (const OptAgent& (ContinuousScene::*) (const planning2d::Id&) const)&ContinuousScene::getOptAgent, return_value_policy<copy_const_reference>())
    .def("getObservations", &getObservationsWrapper, "")
    .def("addObservation", &ContinuousScene::addObservation, "")
    .add_property("minTime", &ContinuousScene::getMinTime)
    .add_property("maxTime", &ContinuousScene::getMaxTime)
    .def("activateAllDesignVariables", &ContinuousScene::activateAllDesignVariables, "None activateAllDesignVariables(bool activate): Activates/Deactivates all spline design variables")
    .add_property("numActiveSplineParameters", &ContinuousScene::numActiveSplineParameters)
    .add_property("activeSplineParameters", &ContinuousScene::activeSplineParameters)
    .def("addDesignVariables", &ContinuousScene::addDesignVariables)
    .def("serialize", &serializeScene, "Serializes the scene to disk. Circumvents pickle which seems to place extra information into the binary output file.")
    .def_pickle(ContinuousScene_pickle_suite<ContinuousScene>())
  ;
  register_ptr_to_python<ContinuousScene::ConstPtr>();
  implicitly_convertible<ContinuousScene::Ptr, ContinuousScene::ConstPtr>();

  class_<ContinuousSceneModifiable, ContinuousSceneModifiable::Ptr, bases<ContinuousScene>, boost::noncopyable>("ContinuousSceneModifiable",
    init<const ContinuousScene&>("ContinuousSceneModifiable(ContinuousScene scene): Constructor"))
    .def(init<>("ContinuousSceneModifiable(): Default constructor"))
    .def("setActiveSplineParameters", &ContinuousSceneModifiable::setActiveSplineParameters)
    .def_pickle(ContinuousScene_pickle_suite<ContinuousSceneModifiable>())
  ;
  register_ptr_to_python<ContinuousSceneModifiable::ConstPtr>();
  implicitly_convertible<ContinuousSceneModifiable::Ptr, ContinuousSceneModifiable::ConstPtr>();

} /* void exportContinuousScene() */
