#ifndef PLANNING2D_POSE2D_IMPLEMENTATION_HPP_
#define PLANNING2D_POSE2D_IMPLEMENTATION_HPP_

// standard includes
#include <cmath> // for std::remainder

// Eigen includes
#include <Eigen/Dense>

// self includes
#include "../Support.hpp"
#include "../Exceptions.hpp"
#include "../Position2d.hpp"

namespace planning2d {

double Pose2d::normalizeMinusPiPlusPi(double val) {
  return std::remainder(val, TWOPI);
}

Pose2d::Pose2d() :
    _pos()
#ifndef NDEBUG
    , _yaw(SIGNAN)
#endif
{

}

Pose2d::Pose2d(const Pose2d::Vector& data) :
  _pos(data.head(2)),
  _yaw(data(2)) {

}

Pose2d::Pose2d(const Position2d& pos, double yaw) :
  _pos(pos),
  _yaw(yaw) {

}

Pose2d::Pose2d(double x, double y, double yaw) :
  _pos(x, y),
  _yaw(yaw) {

}

Pose2d::Pose2d(const Position2d::Vector& xy_, double yaw_) :
  _pos(xy_),
  _yaw(yaw_) {

}

const double& Pose2d::x() const {
  return _pos.x();
}

double& Pose2d::x() {
  return _pos.x();
}

const double& Pose2d::y() const {
  return _pos.y();
}

double& Pose2d::y() {
  return _pos.y();
}

const double& Pose2d::yaw() const {
  return _yaw;
}

double& Pose2d::yaw() {
  return _yaw;
}

Pose2d& Pose2d::normalizeYaw() {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  this->_yaw = Pose2d::normalizeMinusPiPlusPi(_yaw);
  return *this;
}

Position2d& Pose2d::position() {
  return _pos;
}

const Position2d& Pose2d::position() const {
  return _pos;
}

std::size_t Pose2d::dimension() const {
  return Size;
}

const Pose2d::Vector Pose2d::asVector() const {
  Pose2d::Vector mat(this->position().x(), this->position().y(), this->yaw());
  return mat;
}
void Pose2d::setVector(const Vector& data) {
  _pos.setVector(data.head(2));
  _yaw = data(2);
}

bool Pose2d::operator==(const Pose2d& pose) const {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  return this->position() == pose.position() && normalizeMinusPiPlusPi(_yaw) == normalizeMinusPiPlusPi(pose.yaw());
}
bool Pose2d::operator!=(const Pose2d& pose) const {
  return !(*this==pose);
}

Pose2d Pose2d::operator+(const Pose2d& d) const {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  const double c = cos(yaw());
  const double s = sin(yaw());
  const double px = position().x() + c*d.position().x() - s*d.position().y();
  const double py = position().y() + s*d.position().x() + c*d.position().y();
  const double pt = yaw() + d.yaw();
  return Pose2d(px,py,pt);
}

Pose2d Pose2d::operator-(const Pose2d& b) const {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  const double c = cos(b.yaw());
  const double s = sin(b.yaw());
  const double dx = position().x() - b.position().x();
  const double dy = position().y() - b.position().y();
  const double ox =  c*dx + s*dy;
  const double oy = -s*dx + c*dy;
  const double ot = yaw() - b.yaw();
  return Pose2d(ox,oy,ot);
}

inline Pose2d Pose2d::cwisePlus(const Pose2d& d) const {
  return Pose2d(this->position() + d.position(), this->yaw() + d.yaw());
}

inline Pose2d Pose2d::cwiseProduct(const double factor) const {
  return Pose2d(this->position().cwiseProduct(factor), this->yaw() * factor);
}

Position2d Pose2d::transformTo(const Position2d& p) const {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  const double c = cos(yaw());
  const double s = sin(yaw());
  const double dx = p.x() - position().x();
  const double dy = p.y() - position().y();
  const double x =  c*dx + s*dy;
  const double y = -s*dx + c*dy;
  return Position2d(x,y);
}

Position2d Pose2d::transformFrom(const Position2d& p) const {
  SM_ASSERT_FALSE_DBG(InitializationException, std::isnan(this->_yaw),
                      "Yaw is a NaN value.");
  const double c = cos(yaw());
  const double s = sin(yaw());
  const double px = position().x() + c*p.x() - s*p.y();
  const double py = position().y() + s*p.x() + c*p.y();
  return Position2d(px,py);
}

template<class Archive>
void Pose2d::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & _pos;
  ar & _yaw;
}

std::ostream& operator<<(std::ostream& out, const Pose2d& p) {
  return out << "(" << p.position().x() << ", " << p.position().y() << ", " << p.yaw() << ")";
}



Pose2dStamped::Pose2dStamped()
  : Pose2d(),
    StampedType() {
}

Pose2dStamped::Pose2dStamped(const Vector& data, const Time& stamp)
  : Pose2d(data),
    StampedType(stamp) {

}

Pose2dStamped::Pose2dStamped(const Position2d& pos, double yaw, const Time& stamp)
  : Pose2d(pos, yaw),
    StampedType(stamp) {

}

Pose2dStamped::Pose2dStamped(double x, double y, double yaw, const Time& stamp)
  : Pose2d(x, y, yaw),
    StampedType(stamp) {

}

Pose2dStamped::Pose2dStamped(const Pose2d& pose, const Time& stamp)
  : Pose2d(pose),
    StampedType(stamp) {

}

template<class Archive>
inline void Pose2dStamped::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & boost::serialization::base_object<Pose2d>(*this);
  ar & boost::serialization::base_object<StampedType>(*this);
}

} /* namespace planning2d */

#endif /* PLANNING2D_POSE2D_IMPLEMENTATION_HPP_ */
