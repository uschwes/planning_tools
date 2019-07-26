#ifndef PLANNING2D_POSE2D_HPP_
#define PLANNING2D_POSE2D_HPP_

// Eigen includes
#include <Eigen/Geometry>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

// self includes
#include "Support.hpp"
#include "Position2d.hpp"
#include "StampedType.hpp"

namespace planning2d {

class Pose2d {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum { Size = 3 };
  typedef Eigen::Matrix<double,Size,1> Vector;

 public:
  //! Default constructor. In Release mode, lets members uninitialized. In Debug mode, initializes members to signaling_NaN().
  inline Pose2d();
  //! Construct Pose2d from Eigen 3d vector (x,y,yaw)
  inline Pose2d(const Vector& data_);
  //! Construct Pose2d from Position2d plus yaw angle
  inline Pose2d(const Position2d& pos, double yaw_);
  //! Construct Pose2d from x- and y-coordinates plus yaw angle
  inline Pose2d(double x_, double y_, double yaw_);
  //! Construct Pose2d from position vector plus yaw angle
  inline Pose2d(const Position2d::Vector& xy_, double yaw_);
  //! Returns Pose2d as Eigen 3d vector (x,y,yaw)
  inline const Vector asVector() const; // TODO: is there a way not to create a new object?
  //! Sets pose from Eigen 3d vector (x,y,yaw)
  inline void setVector(const Vector& data);
  //! Returns const reference to x
  inline const double& x() const;
  //! Returns mutable reference to x
  inline double& x();
  //! Returns const reference to y
  inline const double& y() const;
  //! Returns mutable reference to y
  inline double& y();
  //! Returns const reference to yaw
  inline const double& yaw() const;
  //! Returns mutable reference to yaw
  inline double& yaw();
  //! Normalizes yaw angle to range [-pi, +pi]
  inline Pose2d& normalizeYaw();
  //! Returns mutable reference to the 2D position
  inline Position2d& position();
  //! Returns const reference to the 2D position
  inline const Position2d& position() const;

  //! Returns the dimension
  inline std::size_t dimension() const;

  //! Equality operator. This operator deals with the identification in angular dimension by normalizing
  //! the yaw angle of both poses before comparison.
  inline bool operator==(const Pose2d& pos) const;
  //! Inequality operator
  inline bool operator!=(const Pose2d& pos) const;

  inline Pose2d operator+(const Pose2d& d) const;
  inline Pose2d operator-(const Pose2d& b) const;
  inline Pose2d cwisePlus(const Pose2d& d) const;
  inline Pose2d cwiseProduct(const double factor) const;
  static inline double normalizeMinusPiPlusPi(double val);

  //! Transforms position expressed in parent of this pose into the frame of this pose
  inline Position2d transformTo(const Position2d& p) const;
  //! Transforms position expressed in the frame of this pose into parent of this pose
  inline Position2d transformFrom(const Position2d& p) const;

  friend inline std::ostream& operator<<(std::ostream& out, const Pose2d& p);

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:

 private:
  Position2d _pos;
  double _yaw;
};

/**
 * @brief Pose with a timestamp
 */
class Pose2dStamped : public Pose2d, public StampedType {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(Pose2dStamped);

  //! Default constructor. In Release mode, lets members uninitialized. In Debug mode, initializes members to signaling_NaN().
  inline Pose2dStamped();
  //! Construct Pose2dStamped from Eigen 3d vector (x,y,yaw) plus stamp
  inline Pose2dStamped(const Vector& data_, const Time& stamp);
  //! Construct Pose2dStamped from Position2d plus yaw angle plus stamp
  inline Pose2dStamped(const Position2d& pos, double yaw_, const Time& stamp);
  //! Construct Pose2dStamped from x- and y-coordinates plus yaw angle plus stamp
  inline Pose2dStamped(double x_, double y_, double yaw_, const Time& stamp);
  //! Construct Pose2dStamped from pose plus stamp
  inline Pose2dStamped(const Pose2d& pose, const Time& stamp);

  //! equality operator
  inline bool operator==(const Pose2dStamped& s) const { return Pose2d::operator==(s) && StampedType::operator==(s); }
  //! inequality operator
  inline bool operator!=(const Pose2dStamped& s) const { return !(*this == s); }

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

}; /* class Pose2dStamped */

} /* namespace planning2d */

#include "planner_interfaces/implementation/Pose2dImplementation.hpp"

#endif /* POSE2D */
