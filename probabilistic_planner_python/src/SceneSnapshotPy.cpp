/*
 * SceneSnapshotPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>

#include <planner_interfaces_python/SupportPy.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;


inline void setObjectContainerWrapper(SceneSnapshot& scene, const SceneSnapshot::StateMap& s) {
  scene.objectContainer() = s;
}

boost::python::dict getObjectContainerWrapper(const SceneSnapshot& scene) {
  boost::python::dict dct;
  for (auto& idObjectPair : scene.objectContainer()) {
    auto& su = idObjectPair.second;
    StateWithUncertainty::Ptr sptr(new StateWithUncertainty(su)); // use pointer because otherwise a copy constructor is called which causes misaligned memory issues. Fixed by using the pointer here and enabling the implicit convertibility of the StateWithUnvertainty and its pointer
    boost::python::object o(sptr);
    dct[idObjectPair.first] = o;
  }
  return dct;
}

void exportSceneSnapshot() {

  class_<SceneSnapshot, SceneSnapshot::Ptr, bases<StampedType> >("SceneSnapshot", init<const planning2d::Time&>("SceneSnapshot(timestamp): constructor"))
      .def(init<const planning2d::Time&>("SceneSnapshot(Time stamp): Constructor"))
      .def("addObject", &SceneSnapshot::addObject)
      .def("getObject", make_function(&SceneSnapshot::getObject, return_internal_reference<>()))
      .add_property("objectContainer", &getObjectContainerWrapper, &setObjectContainerWrapper)
      .add_property("occupancyGrid", make_function((boost::optional<planning2d::OccupancyGrid> (SceneSnapshot::*) (void) const)&SceneSnapshot::getOccupancyGrid, return_value_policy<return_optional>()),
                    &SceneSnapshot::setOccupancyGrid)
  ;

  implicitly_convertible<SceneSnapshot::Ptr, SceneSnapshot::ConstPtr >();

  class_<StateWithUncertainty, StateWithUncertainty::Ptr, bases<State>, boost::noncopyable >("StateWithUncertainty", init<>("StateWithUncertainty(): Default constructor"))
      .def(init<const planning2d::State&, const Eigen::MatrixXd&>("StateWithUncertainty(State state, np.array uncertainty): Constructor"))
      .add_property("invCov", make_function((const Eigen::MatrixXd& (StateWithUncertainty::*) (void) const)&StateWithUncertainty::invCov, return_value_policy<copy_const_reference>()))
  ;
  implicitly_convertible<StateWithUncertainty::Ptr, StateWithUncertainty::ConstPtr >();

} /* void SceneSnapshot() */
