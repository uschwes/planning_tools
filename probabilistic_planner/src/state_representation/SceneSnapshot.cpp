/*
 * SceneSnapshot.cpp
 *
 *  Created on: 04.05.2015
 *      Author: sculrich
 */

#include <planner_interfaces/Exceptions.hpp>

#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"

using namespace prob_planner;

/**
 * Constructor
 * @param stamp timestamp of the snapshot
 */
SceneSnapshot::SceneSnapshot(const planning2d::Time& stamp)
    : StampedType(stamp) { }

SceneSnapshot::SceneSnapshot(const planning2d::Time& stamp, std::initializer_list< std::pair<const planning2d::Id,StateWithUncertainty> > l)
    : StampedType(stamp), _objectContainer(l) { }

SceneSnapshot::SceneSnapshot(const planning2d::Time& stamp, const planning2d::Id id, const StateWithUncertainty& meas)
    : SceneSnapshot::SceneSnapshot(stamp, {std::make_pair(id, meas) } ) { }

SceneSnapshot::SceneSnapshot(const planning2d::Time& stamp, const planning2d::OccupancyGrid& grid)
    : StampedType(stamp), _occupancyGrid(grid) { }

/**
 *
 * @param id
 * @param state
 */
void SceneSnapshot::addObject(const planning2d::Id& id, const StateWithUncertainty& state) {
  _objectContainer.insert(StateMap::value_type(id, state));
}

bool SceneSnapshot::hasObject(const planning2d::Id& id) const {
  return _objectContainer.find(id) != _objectContainer.end();
}

/**
 * Return an object added before
 * @param id id of object
 * @return
 */
const StateWithUncertainty& SceneSnapshot::getObject(const planning2d::Id& id) const {
  StateMap::const_iterator it = _objectContainer.find(id);
  SM_ASSERT_TRUE(planning2d::LookupException, it != _objectContainer.end(), "Object with id " << id << " does not exist.");
  return it->second;
}
