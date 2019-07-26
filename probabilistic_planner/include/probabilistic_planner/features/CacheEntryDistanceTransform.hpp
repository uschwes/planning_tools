/*
 * CacheEntryDistanceTransform.hpp
 *
 *  Created on: 18.12.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_CACHEENTRYDISTANCETRANSFORM_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_CACHEENTRYDISTANCETRANSFORM_HPP_

// planner interfaces
#include <planner_interfaces/Exceptions.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>

// cache
#include <simple_cache/CacheEntryInterface.hpp>

// self
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>

// planner_algorithms
#include <planner_algorithms/DistanceTransform.hpp>

namespace prob_planner {

/// \brief A cache entry for the distance transform of the occupancy grid measurement
struct CacheEntryDistanceTransform : public cache::CacheEntryInterface
{
  CacheEntryDistanceTransform() { }
  CacheEntryDistanceTransform(const planning2d::OccupancyGrid& grid)
      : distanceTransform(new planning2d::Map<float>(grid.getOrigin(), grid.resolution(), grid.sizeInCells()))
  {
    compute(grid);
  }
  virtual ~CacheEntryDistanceTransform() { }
  void compute(const planning2d::OccupancyGrid& grid) {
    planning2d::algorithms::signedDistanceTransform(grid, *distanceTransform);
    setValid(true);
  }
  bool isValid() const override { return _isValid; }
  void setValid(const bool valid) { _isValid = valid; }

  planning2d::Map<float>::Ptr distanceTransform;

  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    boost::serialization::void_cast_register<CacheEntryDistanceTransform,cache::CacheEntryInterface>();
    ar & distanceTransform;
    ar & _isValid;
  }
 private:
  bool _isValid = true;
};

/// \brief Computes or retrieves the cache entries for the signed distance transform of a grid stored in a scene snapshot
inline void retrieveOrCreateCacheEntrySignedDistanceTransform(const SceneSnapshot& snapshot, planning2d::Map<float>::Ptr& sdt)
{
  SM_ASSERT_TRUE(planning2d::FunctionInputException, snapshot.getOccupancyGrid(), "");
  static const std::string cacheEntryName = "occupancy_grid_signed_distance_transform";
  auto cachebox = snapshot.cache().get(cacheEntryName);
  boost::shared_ptr<CacheEntryDistanceTransform> dtCache;
  if (cachebox->second == nullptr) {
    dtCache = boost::shared_ptr<CacheEntryDistanceTransform>(new CacheEntryDistanceTransform(snapshot.getOccupancyGrid().get()));
    cachebox->second = dtCache;
    SM_VERBOSE_STREAM_NAMED("caching", "Creating new cache entry for \"" << cacheEntryName << "\"");
  } else {
    dtCache = boost::dynamic_pointer_cast<CacheEntryDistanceTransform>(cachebox->second);
    SM_ASSERT_TRUE(planning2d::RuntimeException, dtCache != nullptr, "Cache retrieval for \"" << cacheEntryName << "\" failed");
    if (!dtCache->isValid())
      dtCache->compute(snapshot.getOccupancyGrid().get());
    SM_VERBOSE_STREAM_NAMED("caching", "Reusing cache entry for \"" << cacheEntryName << "\"");
  }
  sdt = dtCache->distanceTransform;
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_CACHEENTRYDISTANCETRANSFORM_HPP_ */
