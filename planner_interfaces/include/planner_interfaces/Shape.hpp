#ifndef PLANNER_INTERFACE_SHAPE_HPP
#define PLANNER_INTERFACE_SHAPE_HPP

// standard includes
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

// FCL forward declarations
// Use #include <fcl/shape/geometric_shapes.h> to include them into a CU.
namespace fcl {
	class CollisionObject;
	class CollisionGeometry;
}

// self includes
#include "Position2d.hpp"
#include "Pose2d.hpp"

namespace planning2d {

typedef fcl::CollisionObject CollisionObject;
typedef boost::shared_ptr<CollisionObject> CollisionObjectPtr;
typedef boost::shared_ptr<const CollisionObject> CollisionObjectConstPtr;

typedef fcl::CollisionGeometry CollisionGeometry;
typedef boost::shared_ptr<CollisionGeometry> CollisionGeometryPtr;
typedef boost::shared_ptr<const CollisionGeometry> CollisionGeometryConstPtr;

/**
 * @brief Approximation of arbitrary shape with a collection of discs
 */
class DiscApproximation {
 public:

  //! Default constructor
  inline DiscApproximation();

  //! Constructor initializing memory for \var nDisks
  inline DiscApproximation(const std::size_t nDisks);

  //! Get radius of disc number \var i
  inline double getRadius(std::size_t i) const;

  //! Get position of disc number \var i
  inline const Position2d& getPosition(std::size_t i) const;

  /**
   * Returns the number of discs in the data structure
   * @return Number of discs
   */
  inline std::size_t getNumDiscs() const;

  /**
   * Inserts a disc into the data structure
   * @param[in] radius The radius of the disc
   * @param[in] position Position of the disc' center point in the coordinate frame of the agent
   */
  inline void insertDisc(double radius, const Position2d& position);

  /**
   * Transform all the contained discs from the frame given by \var pose
   * @param pose Frame to transform from
   * @return Transformed disc approximation object
   */
  inline DiscApproximation& transformFrom(const Pose2d& pose);

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) { ar & _radii & _positions; }

 private:
  std::vector<double> _radii; //! radii of the discs
  std::vector<Position2d> _positions; //! Positions of the discs' center points in the coordinate frame of the agent
};

} /* namespace planning2d */

#include "implementation/ShapeImplementation.hpp"

#endif /* PLANNER_INTERFACE_SHAPE_HPP */
