/*
 * StampedType.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_STAMPEDTYPE_HPP_
#define PLANNING2D_STAMPEDTYPE_HPP_

// self includes
#include "Time.hpp"

namespace planning2d {

class StampedType {

 public:

  inline StampedType() { }
  inline StampedType(Time stamp) : _stamp(stamp) { }
  inline StampedType(const StampedType& s) : _stamp(s.stamp()) { }

  //! Returns const reference to stamp
  inline const Time& stamp() const { return _stamp;}
  //! Returns mutable reference to stamp
  inline Time& stamp() { return _stamp; }

  //! equality operator
  inline bool operator==(const StampedType& t) const { return _stamp == t.stamp(); }
  //! inequality operator
  inline bool operator!=(const StampedType& t) const { return !(*this == t); }

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) { ar & _stamp; }

 private:
  Time _stamp; //! timestamp

}; /* class StampedType */

struct StampedTypeComparator {
  bool operator()(const StampedType& st0, const StampedType& st1) const {
    return st0.stamp() < st1.stamp();
  }
};


} /* namespace planning2d */

#endif /* PLANNING2D_STAMPEDTYPE_HPP_ */
