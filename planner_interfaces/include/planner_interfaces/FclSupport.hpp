/*
 * FclSupport.hpp
 *
 *  Created on: 24.02.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PLANNER_INTERFACES_FCLSUPPORT_HPP_
#define INCLUDE_PLANNER_INTERFACES_FCLSUPPORT_HPP_

#include <fcl/shape/geometric_shapes.h>

#define REGISTER_FCL_TYPES_SERIALIZATION(ar) \
  ar.template register_type<fcl::Box>(); \
  ar.template register_type<fcl::Cylinder>()

BOOST_SERIALIZATION_SPLIT_FREE(fcl::Box);

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, fcl::CollisionGeometry& c, const unsigned int /*version*/)
{

}

template<class Archive, typename T>
void serialize(Archive & ar, fcl::details::Vec3Data<T>& c, const unsigned int /*version*/)
{
  ar & c.vs;
}

template<class Archive>
void serialize(Archive & ar, fcl::Vec3f& c, const unsigned int /*version*/)
{
  ar & c.data;
}

template<class Archive>
void save(Archive & ar, const fcl::Box& c, const unsigned int /*version*/)
{
  boost::serialization::void_cast_register<fcl::Box, fcl::CollisionGeometry>();
  double x = c.side[0];
  double y = c.side[1];
  double z = c.side[2];
  ar << x;
  ar << y;
  ar << z;
}

template<class Archive>
void load(Archive & ar, fcl::Box& c, const unsigned int /*version*/)
{
  boost::serialization::void_cast_register<fcl::Box, fcl::CollisionGeometry>();
  double x, y, z;
  ar >> x;
  ar >> y;
  ar >> z;
  c.side = fcl::Vec3f(x, y, z);
}

template<class Archive>
void serialize(Archive & /*ar*/, fcl::Cylinder& /*c*/, const unsigned int /*version*/)
{
  boost::serialization::void_cast_register<fcl::Cylinder, fcl::CollisionGeometry>();
}

template<class Archive>
void save_construct_data(Archive & ar, const fcl::Cylinder* c, const unsigned int /*version*/)
{
    ar << c->radius;
    ar << c->lz;
}
template<class Archive>
void load_construct_data(Archive & ar, fcl::Cylinder* c, const unsigned int /*version*/)
{
    double radius, lz;
    ar >> radius;
    ar >> lz;
    ::new(c)fcl::Cylinder(radius, lz);
}

} // namespace serialization
} // namespace boost

#endif /* INCLUDE_PLANNER_INTERFACES_FCLSUPPORT_HPP_ */
