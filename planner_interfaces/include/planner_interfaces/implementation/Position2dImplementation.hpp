/*
 * Point2dImplementation.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_POSITION2D_IMPLEMENTATION_HPP_
#define PLANNING2D_POSITION2D_IMPLEMENTATION_HPP_

// standard includes
#include <cmath>
#include <functional>

// boost
#include <boost/type_traits/is_integral.hpp>

// self includes
#include "../Support.hpp"
#include "../Exceptions.hpp"

namespace planning2d {

  template <typename T>
  T floor(T val) {
    if (boost::is_integral<T>::value) // overload to return integer type for integers
      return val;
    else
      return std::floor(val);
  }

  template <typename T>
  T ceil(T val) {
    if (boost::is_integral<T>::value) // overload to return integer type for integers
      return val;
    else
      return std::ceil(val);
  }


  template <typename T>
  Point2d<T>::Point2d()
#ifndef NDEBUG
    : _xy(SIGNAN,SIGNAN)
#endif
  {

  }

  template <typename T>
  Point2d<T>::Point2d(const Vector& data) : _xy(data) {

  }

  template <typename T>
  Point2d<T>::Point2d(T x, T y) :
      _xy(x,y) {

  }

  template <typename T>
  Point2d<T>::Point2d(const Point2d<T>& p) :
    _xy(p.x(), p.y()) {

  }

  template <typename T>
  inline Point2d<T> Point2d<T>::Zero() {
    return Point2d((T)0, (T)0);
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::Constant(const T& val) {
    return Point2d(val, val);
  }

  template <typename T>
  const T& Point2d<T>::x() const {
    return _xy(0);
  }

  template <typename T>
  T& Point2d<T>::x() {
    return _xy(0);
  }

  template <typename T>
  const T& Point2d<T>::y() const {
    return _xy(1);
  }

  template <typename T>
  T& Point2d<T>::y() {
    return _xy(1);
  }

  template <typename T>
  std::size_t Point2d<T>::dimension() const {
    return Size;
  }

  template <typename T>
  const typename Point2d<T>::Vector& Point2d<T>::asVector() const {
    return this->_xy;
  }

  template <typename T>
  inline Point2d<T>::operator const Vector&() const {
    return this->_xy;
  }

  template <typename T>
  void Point2d<T>::setVector(const Vector& xy) {
    this->_xy = xy;
  }

  template <typename T>
  template <typename D>
  inline Point2d<D> Point2d<T>::cast() const {
    return Point2d<D>(this->_xy.cast<D>());
  }

  template <typename T>
  inline Point2d<T>& Point2d<T>::floor() {
    this->_xy = this->_xy.unaryExpr(&::planning2d::floor<T>);
    return *this;
  }

  template <typename T>
  inline Point2d<T>& Point2d<T>::ceil() {
    this->_xy = this->_xy.unaryExpr(&::planning2d::ceil<T>);
    return *this;
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::floor() const {
    return Point2d<T>(this->_xy.unaryExpr(&::planning2d::floor<T>));
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::ceil() const {
    return Point2d<T>(this->_xy.unaryExpr(&::planning2d::ceil<T>));
  }

  template <typename T>
  bool Point2d<T>::operator==(const Point2d<T>& pos) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    SM_ASSERT_TRUE_DBG(InitializationException, (pos.asVector().isApprox(pos.asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return this->asVector() == pos.asVector();
  }

  template <typename T>
  bool Point2d<T>::operator!=(const Point2d<T>& pos) const {
    return !(*this==pos);
  }

  template <typename T>
  bool Point2d<T>::operator>(const Point2d<T>& pos) const {
    return (this->asVector().array() > pos.asVector().array()).all();
  }

  template <typename T>
  bool Point2d<T>::operator<(const Point2d<T>& pos) const {
    return (this->asVector().array() < pos.asVector().array()).all();
  }

  template <typename T>
  bool Point2d<T>::operator>=(const Point2d<T>& pos) const {
    return (this->asVector().array() >= pos.asVector().array()).all();
  }

  template <typename T>
  bool Point2d<T>::operator<=(const Point2d<T>& pos) const {
    return (this->asVector().array() <= pos.asVector().array()).all();
  }

  template <typename T>
  Point2d<T> Point2d<T>::operator+(const Point2d<T>& pos) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return Point2d(this->asVector().array() + pos.asVector().array());
  }

  template <typename T>
  Point2d<T>& Point2d<T>::operator+=(const Point2d<T>& pos) {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    this->_xy.array() += pos.asVector().array();
    return *this;
  }

  template <typename T>
  Point2d<T> Point2d<T>::operator-(const Point2d<T>& pos) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return Point2d(this->asVector().array() - pos.asVector().array());
  }

  template <typename T>
  Point2d<T>& Point2d<T>::operator-=(const Point2d<T>& pos) {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    this->_xy.array() -= pos.asVector().array();
    return *this;
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::operator+(const T s) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return Point2d(this->asVector().array() + s);
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::operator-(const T s) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return Point2d(this->asVector().array() - s);
  }

  template <typename T>
  inline void Point2d<T>::setZero() {
    this->_xy.setZero();
  }

  template <typename T>
  inline void Point2d<T>::setConstant(const T val) {
    this->_xy.setConstant(val);
  }

  template <typename T>
  inline Point2d<T> Point2d<T>::cwiseProduct(const double factor) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return Point2d(this->asVector() * factor);
  }

  template <typename T>
  double Point2d<T>::norm() const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return this->asVector().norm();
  }

  template <typename T>
  double Point2d<T>::squaredNorm() const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return this->asVector().squaredNorm();
  }

  template <typename T>
  double Point2d<T>::sum() const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->asVector().isApprox(this->asVector())),
                       "The point contains at least one NaN value, not initialized?");
    return this->asVector().sum();
  }

  template <typename T>
  template<class Archive>
  inline void Point2d<T>::serialize(Archive & ar, const unsigned int /*version*/) {
    ar & _xy;
  }

  template <typename T>
  std::ostream& operator<<(std::ostream& out, const Point2d<T>& p) {
    return out << "(" << p.x() << ", " << p.y() << ")";
  }


  template <typename T>
  Point2dStamped<T>::Point2dStamped()
    : Point2d<T>(),
      StampedType() {

  }

  template <typename T>
  Point2dStamped<T>::Point2dStamped(const typename Point2d<T>::Vector& data, const Time& stamp)
    : Point2d<T>(data),
      StampedType(stamp) {

  }

  template <typename T>
  Point2dStamped<T>::Point2dStamped(T x, T y, const Time& stamp)
    : Point2d<T>(x, y),
      StampedType(stamp) {

  }

  template <typename T>
  Point2dStamped<T>::Point2dStamped(const Point2dStamped<T>& p)
    : Point2d<T>(p),
      StampedType(p.stamp()) {

  }

  template <typename T>
  Point2dStamped<T>::Point2dStamped(const Point2d<T>& point, const Time& stamp)
    : Point2d<T>(point),
      StampedType(stamp) {

  }

  template <typename T>
  template<class Archive>
  void Point2dStamped<T>::serialize(Archive & ar, const unsigned int /*version*/) {
    ar & boost::serialization::base_object< Point2d<T> >(*this);
    ar & boost::serialization::base_object<StampedType>(*this);
  }

  template <typename T>
  std::ostream& operator<<(std::ostream& out, const Point2dStamped<T>& p) {
    return out << "(" << p.stamp() << ": " << p.x() << ", " << p.y() << ")";
  }




  template <typename T>
  inline double distance(const Point2d<T>& p0, const Point2d<T>& p1) {
    SM_ASSERT_TRUE_DBG(InitializationException, (p0.asVector().isApprox(p0.asVector())),
                       "The point p0 contains at least one NaN value.");
    SM_ASSERT_TRUE_DBG(InitializationException, (p1.asVector().isApprox(p1.asVector())),
                       "The point p1 contains at least one NaN value.");
    return (p0-p1).norm();
  }

} /* namespace planning_2d */


#endif /* PLANNING2D_POSITION2D_IMPLEMENTATION_HPP_ */
