/*
 * Position2d.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_POSITION2D_HPP_
#define PLANNING2D_POSITION2D_HPP_

// Eigen includes
#include <Eigen/Dense>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

// self includes
#include "Support.hpp"
#include "StampedType.hpp"

namespace planning2d {

template <class T>
class Point2d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PLANNING_2D_POINTER_TYPEDEFS(Point2d<T>);
  enum { Size = 2 };
  typedef Eigen::Matrix<T,Size,1,Eigen::DontAlign> Vector; // DontAlign added because of http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html"
  typedef T Scalar;

 public:
  //! Default constructor. In Release mode, lets members uninitialized. In Debug mode, initializes members to signaling_NaN().
  inline Point2d();
  //! Construct Point2d from Eigen 2d vector.
  inline Point2d(const Vector& data_);
  //! Construct Point2d from x and y coordinates
  inline Point2d(T x_, T y_);
  //! Copy constructor
  inline Point2d(const Point2d<T>& p);
  //! Creates a Point2d with zero entries
  inline static Point2d<T> Zero();
  //! Creates a Point2d with constant entries
  inline static Point2d<T> Constant(const T& val);
  //! Returns Point2d as Eigen 2d vector
  inline const Vector& asVector() const;
  //! Implicit cast operator to Eigen 2d vector
  inline operator const Vector&() const;
  //! Sets position from Eigen 2d vector
  inline void setVector(const Vector& data);
  //! Cast to different type
  template <typename D> inline Point2d<D> cast() const;
  //! Returns const reference to the x-coordinate
  inline const T& x() const;
  //! Returns mutable reference to the x-coordinate
  inline T& x();
  //! Returns const reference to the y-coordinate
  inline const T& y() const;
  //! Returns mutable reference to the y-coordinate
  inline T& y();

  //! The dimensionality of the point
  inline std::size_t dimension() const;

  //! Equality operator
  inline bool operator==(const Point2d<T>& pos) const;
  //! Inequality operator
  inline bool operator!=(const Point2d<T>& pos) const;
  //! Checks whether both coordinates of this are larger
  inline bool operator>(const Point2d<T>& pos) const;
  //! Checks whether both coordinates of this are smaller
  inline bool operator<(const Point2d<T>& pos) const;
  //! Checks whether both coordinates of this are larger or equal
  inline bool operator>=(const Point2d<T>& pos) const;
  //! Checks whether both coordinates of this are smaller or equal
  inline bool operator<=(const Point2d<T>& pos) const;

  //! Returns addition of this Point2d and pos
  inline Point2d<T> operator+(const Point2d<T>& pos) const;
  //! Adds position to this Point2d object
  inline Point2d<T>& operator+=(const Point2d<T>& pos);
  //! Returns subtraction of this Point2d and pos
  inline Point2d<T> operator-(const Point2d<T>& pos) const;
  //! Subtracts position to this Point2d object
  inline Point2d<T>& operator-=(const Point2d<T>& pos);
  //! Multiplication with scalar
  inline Point2d<T> cwiseProduct(const double factor) const;

  //! Returns addition of this Point2d and a scalar for each entry
  inline Point2d<T> operator+(const T s) const;
  //! Returns subtraction of this Point2d and a scalar for each entry
  inline Point2d<T> operator-(const T s) const;

  //! Sets all values to zero
  void setZero();
  //! Sets all values to a constant
  void setConstant(const T val);

  //! For both x and y, computes the largest integer value not greater than the entry
  inline Point2d<T>& floor();
  //! For both x and y, computes the largest integer value not less than the entry
  inline Point2d<T>& ceil();
  //! For both x and y, computes the largest integer value not greater than the entry
  inline Point2d<T> floor() const;
  //! For both x and y, computes the largest integer value not less than the entry
  inline Point2d<T> ceil() const;

  //! Computes norm
  inline double norm() const;
  //! Computes squared norm
  inline double squaredNorm() const;
  //! Computes the sum (Manhattan distance) of the entries
  inline double sum() const;

  //! Stream operator overload for Point2d
  template <typename D>
  friend inline std::ostream& operator<<(std::ostream& out, const Point2d<D>& p);

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 protected:
  Vector _xy;

}; /* class Point2d */

/**
 * @brief Point2d with a timestamp
 */
template <typename T>
class Point2dStamped : public Point2d<T>, public StampedType {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(Point2dStamped<T>);

  //! Default constructor. In Release mode, lets members uninitialized. In Debug mode, initializes members to signaling_NaN().
  inline Point2dStamped();
  //! Construct Point2dStamped from Eigen 2d vector.
  inline Point2dStamped(const typename Point2d<T>::Vector& data, const Time& stamp);
  //! Construct Point2dStamped from x and y coordinates
  inline Point2dStamped(T x, T y, const Time& stamp);
  //! Copy constructor
  inline Point2dStamped(const Point2dStamped<T>& p);
  //! Construct Point2dStamped from point plus stamp
  inline Point2dStamped(const Point2d<T>& point, const Time& stamp);

  //! equality operator
  inline bool operator==(const Point2dStamped<T>& s) const { return Point2d<T>::operator==(s) && StampedType::operator==(s); }
  //! inequality operator
  inline bool operator!=(const Point2dStamped<T>& s) const { return !(*this == s); }

  //! Stream operator overload for Point2dStamped
  template <typename D>
  friend inline std::ostream& operator<<(std::ostream& out, const Point2dStamped<D>& p);

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

}; /* class Point2dStamped */

template <typename T>
inline double distance(const Point2d<T>& p0, const Point2d<T>& p1);

typedef Point2d<double> Position2d;
typedef Point2dStamped<double> Position2dStamped;

} /* namespace planning_2d */

#include "implementation/Position2dImplementation.hpp"

#endif /* PLANNING2D_POSITION2D_HPP_ */
