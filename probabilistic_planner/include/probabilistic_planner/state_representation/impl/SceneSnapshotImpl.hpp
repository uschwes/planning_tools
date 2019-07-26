/*
 * SceneSnapshotImpl.hpp
 *
 *  Created on: 03.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_SCENESNAPSHOTIMPL_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_SCENESNAPSHOTIMPL_HPP_

#include <boost/serialization/optional.hpp>
#include <boost/serialization/map.hpp>
#include <sm/eigen/serialization.hpp>

namespace prob_planner {

template<class Archive>
inline void StateWithUncertainty::serialize(Archive & ar, const unsigned int /*version*/)
{
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(planning2d::State);
  ar & _invCov;
}

template<class Archive>
inline void SceneSnapshot::serialize(Archive & ar, const unsigned int /*version*/)
{
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(planning2d::StampedType);
  ar & _occupancyGrid;
  ar & _objectContainer;
  ar & _cache;
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_SCENESNAPSHOTIMPL_HPP_ */
