/*
 * SupportPy.hpp
 *
 *  Created on: Feb 23, 2015
 *      Author: sculrich
 */

#ifndef SRC_SUPPORTPY_HPP_
#define SRC_SUPPORTPY_HPP_

#include <boost/optional.hpp>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/integral_constant.hpp>
#include <boost/utility/in_place_factory.hpp>

template <typename T>
std::string toString(const T& val) {
  std::ostringstream os;
  os << val;
  return os.str();
}

// taken from http://stackoverflow.com/questions/26497922/how-to-wrap-a-c-function-that-returns-boostoptionalt
namespace detail {

template <typename T>
struct is_optional : boost::false_type {};

template <typename T>
struct is_optional<boost::optional<T> > : boost::true_type {};

template <typename>
struct return_optional_requires_a_optional_return_type {};

template <typename T>
struct to_python_optional
{
  bool convertible() const { return ::detail::is_optional<T>::value; }
  PyObject* operator()(const T& obj) const {
    boost::python::object result = obj ? boost::python::object(*obj) : boost::python::object();    // Otherwise, return Python None.
    return boost::python::incref(result.ptr());
  }
  const PyTypeObject* get_pytype() const { return 0; }
};

} // namespace detail

struct return_optional
{
  template <class T> struct apply {
    typedef typename boost::mpl::if_<
      ::detail::is_optional<T>,
      ::detail::to_python_optional<T>,
      ::detail::return_optional_requires_a_optional_return_type<T>
    >::type type;
  }; // apply
};   // return_optional

#endif /* SRC_SUPPORTPY_HPP_ */
