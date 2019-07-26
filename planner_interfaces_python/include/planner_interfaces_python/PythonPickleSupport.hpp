/*
 * PythonPickleSupport.hpp
 *
 *  Created on: 26.11.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PLANNER_INTERFACES_PYTHON_PYTHONPICKLESUPPORT_HPP_
#define INCLUDE_PLANNER_INTERFACES_PYTHON_PYTHONPICKLESUPPORT_HPP_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <boost/python/object/pickle_support.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

/**
 * @struct DefaultTypeRegistry
 * Noop type registry object
 * @tparam Archive Input/Output archive type
 */
template <typename Archive>
struct DefaultTypeRegistry {
  static void registerTypes(Archive& /*a*/) { }
};

/**
 * @struct BoostSerialization_pickle_suite
 * Boost python pickle suite using boost serialization
 * @tparam T Object to pickle
 * @tparam IArchive input archive type
 * @tparam OArchive output archive type
 * @tparam TypeRegistry Optional type registry object for polymorphic serialization
 */
template <typename T, typename IArchive, typename OArchive, template<typename> class TypeRegistry = DefaultTypeRegistry>
struct BoostSerialization_pickle_suite : boost::python::pickle_suite {
  typedef IArchive iarchive;
  typedef OArchive oarchive;
  static boost::python::object getstate (const T& val) {
    std::ostringstream os;
    oarchive oa(os);
    TypeRegistry<oarchive>::registerTypes(oa);
    oa << val;
    return boost::python::str (os.str());
  }
  static void setstate(T& val, boost::python::object entries) {
    boost::python::str s = boost::python::extract<boost::python::str> (entries)();
    std::string st = boost::python::extract<std::string> (s)();
    std::istringstream is (st);
    iarchive ia(is);
    TypeRegistry<iarchive>::registerTypes(ia);
    ia >> val;
  }
};

/**
 * @struct BoostSerializationBinary_pickle_suite
 * Boost python pickle suite using boost binary serialization
 * @tparam T Object to pickle
 * @tparam TypeRegistry Optional type registry object for polymorphic serialization
 */
template <typename T, template<typename> class TypeRegistry = DefaultTypeRegistry>
struct BoostSerializationBinary_pickle_suite :
    public BoostSerialization_pickle_suite<T, boost::archive::binary_iarchive, boost::archive::binary_oarchive, TypeRegistry> { };

/**
 * Creates a shared pointer to a vector from a python list
 * Useful to create a constructor for a vector e.g. wrapped with indexing_suite.
 * @param l boost python list
 * @return shared pointer to vector
 */
template <typename T>
boost::shared_ptr< std::vector<T> > vector_shared_ptr_from_list(const boost::python::list& l) {
  typedef std::vector<T> vec;
  boost::shared_ptr<vec> ret(new vec());
  for (int i=0; i<len(l); ++i) {
    boost::python::extract<T> extractor(l[i]);
    if (!extractor.check()) { throw std::runtime_error("Wrong type provided"); }
    ret->push_back(extractor());
  }
  return ret;
}

/**
 * Creates a shared pointer to a vector from a python object.
 * Useful to create a constructor for a vector e.g. wrapped with indexing_suite.
 * @param obj python object
 * @return shared pointer to vector
 */
template <typename T>
boost::shared_ptr< std::vector<T> > vector_shared_ptr_from_object(const T& obj) {
  typedef std::vector<T> vec;
  return boost::shared_ptr<vec>(new vec(1, obj));
}

#endif /* INCLUDE_PLANNER_INTERFACES_PYTHON_PYTHONPICKLESUPPORT_HPP_ */
