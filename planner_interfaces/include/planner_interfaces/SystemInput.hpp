/*
 * SystemInput.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2d_SYSTEMINPUT_HPP_
#define PLANNING2d_SYSTEMINPUT_HPP_

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

// self includes
#include "Support.hpp"
#include "Pose2d.hpp"
#include "Time.hpp"
#include "StampedType.hpp"

namespace planning2d {

class SystemInput {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(SystemInput);
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> T;

  //! default constructor
  inline SystemInput() { }
  //! Constructor allocating memory for \var sz variables
  inline SystemInput(std::size_t sz) { _inputs.resize(sz); }
  //! Constructor initializing with Eigen vector
  inline SystemInput(const T& inputs) : _inputs(inputs) { }
  //! Destructor
  virtual ~SystemInput() { }

  //! returns dimension of the input, i.e. number of variables
  inline std::size_t dimension() const { return _inputs.rows(); }
  //! returns const reference to input variable \var i
  inline const double& operator()(std::size_t i) const { return _inputs(i); }
  //! returns mutable reference to input variable \var i
  inline double& operator()(std::size_t i) { return _inputs(i); }
  //! returns const reference to input Eigen vector
  inline const T& data() const { return _inputs; }
  //! returns mutable reference to input Eigen vector
  inline T& data() { return _inputs; }

  //! equality operator
  inline bool operator==(const SystemInput& state) const { return _inputs == state.data(); }
  //! inequality operator
  inline bool operator!=(const SystemInput& state) const { return !(*this == state); }

  //! Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) { ar & _inputs; }

 private:
  T _inputs; //! data structure holding the input vector

}; /* class SystemInput */


class SystemInputStamped : public virtual SystemInput, public virtual StampedType {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(SystemInputStamped);

  inline SystemInputStamped() : SystemInput() { }
  inline SystemInputStamped(std::size_t sz) : SystemInput(sz) { }
  inline SystemInputStamped(std::size_t sz, const Time& stamp) : SystemInput(sz), StampedType(stamp) { }
  inline SystemInputStamped(const SystemInput::T& inputs, const Time& stamp) : SystemInput(inputs), StampedType(stamp) { }
  inline SystemInputStamped(const SystemInput& input, const Time& stamp) : SystemInput(input), StampedType(stamp) { }
  virtual ~SystemInputStamped() { }

  //! equality operator
  inline bool operator==(const SystemInputStamped& u) const { return SystemInput::operator==(u) && StampedType::operator==(u); }
  //! inequality operator
  inline bool operator!=(const SystemInputStamped& u) const { return !(*this == u); }

  //! Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & boost::serialization::base_object<SystemInput>(*this);
    ar & boost::serialization::base_object<StampedType>(*this);
  }

}; /* class SystemInputStamped */

} /* namespace planning2d */

#endif /* PLANNING2d_SYSTEMINPUT_HPP_ */
