#ifndef SHAPE2D_IMPLEMENTATION_HPP_
#define SHAPE2D_IMPLEMENTATION_HPP_

// self includes
#include "../Support.hpp"

namespace planning2d {

//! Default constructor
inline DiscApproximation::DiscApproximation() { }

//! Constructor initializing memory for \var nDisks
inline DiscApproximation::DiscApproximation(const std::size_t nDisks) {
  _radii.reserve(nDisks);
  _positions.reserve(nDisks);
}

//! Get radius of disc number \var i
inline double DiscApproximation::getRadius(const std::size_t i) const { return _radii[i]; }

//! Get position of disc number \var i
inline const Position2d& DiscApproximation::getPosition(std::size_t i) const { return _positions[i]; }

/**
 * Returns the number of discs in the data structure
 * @return Number of discs
 */
inline std::size_t DiscApproximation::getNumDiscs() const { return _radii.size(); }

/**
 * Inserts a disc into the data structure
 * @param[in] radius The radius of the disc
 * @param[in] position Position of the disc' center point in the coordinate frame of the agent
 */
inline void DiscApproximation::insertDisc(const double radius, const Position2d& position) {
  _radii.push_back(radius);
  _positions.push_back(position);
}

/**
 * Transform all the contained discs from the frame given by \var pose
 * @param pose Frame to transform from
 * @return Transformed disc approximation object
 */
inline DiscApproximation& DiscApproximation::transformFrom(const Pose2d& pose) {
  for (std::size_t i=0; i<this->getNumDiscs(); i++) {
    this->_positions[i] = pose.transformFrom(this->_positions[i]);
  }
  return *this;
}

} /* namespace planning2d */

#endif /* SHAPE2D_IMPLEMENTATION_HPP_ */
