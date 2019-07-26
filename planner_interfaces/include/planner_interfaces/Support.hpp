#ifndef PLANNER_INTERFACES_SUPPORT_HPP_
#define PLANNER_INTERFACES_SUPPORT_HPP_

// standard includes
#include <limits>

// Schweizer Messer includes
#include <sm/PropertyTree.hpp>
#include <sm/timing/Timer.hpp>

// Eigen
#include <Eigen/Geometry>

#define PLANNING_2D_POINTER_TYPEDEFS(TYPE)  \
  typedef boost::shared_ptr<TYPE> Ptr; \
  typedef boost::shared_ptr<const TYPE> ConstPtr

#ifndef SIGNAN
  #define SIGNAN std::numeric_limits<double>::signaling_NaN()
#endif

#define PLANNING_2D_DECLARE_CONSTRUCTORS(CLASSNAME) \
  private: \
    bool m_isInitialized; \
  public: \
    CLASSNAME(); \
    CLASSNAME(const sm::ConstPropertyTree& config); \
    static CLASSNAME::Ptr make_ptr(const sm::ConstPropertyTree& config) { \
      return boost::shared_ptr<CLASSNAME>(new CLASSNAME(config)); \
    } \
    static CLASSNAME::Ptr make_ptr() { \
      return boost::shared_ptr<CLASSNAME>(new CLASSNAME()); \
    } \
    void initialize(const sm::ConstPropertyTree& config); \
    inline bool isInitialized() const { \
      return m_isInitialized; \
    }
  
// This should be used in the private: declarations for a class
#define PLANNING_2D_DISALLOW_COPY_AND_ASSIGN(CLASSNAME) \
  CLASSNAME(const CLASSNAME&);               \
  void operator=(const CLASSNAME&)

#if !defined(LIKELY) || !defined(UNLIKELY)
  #if defined(__GNUC__) || defined(__GNUG__)  /* GNU GCC/G++ */
    #define LIKELY(x)    __builtin_expect (!!(x), 1)
    #define UNLIKELY(x)  __builtin_expect (!!(x), 0)
  #else
    #define LIKELY(x)    x
    #define UNLIKELY(x)  x
  #endif
#endif

#ifdef planner_interfaces_ENABLE_TIMING
  typedef sm::timing::Timer Timer;
#else
  typedef sm::timing::DummyTimer Timer;
#endif

namespace Eigen {

//! Overloaded stream operator for Eigen::AlignedBox
template <typename Scalar, int Dim>
std::ostream& operator<<(std::ostream& os, const Eigen::AlignedBox<Scalar, Dim>& box)
{
  static const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ", ", "", "", "(", ")");
  os << std::fixed
     << "[" << box.min().format(fmt) << "," << box.max().format(fmt) << "]";
  return os;
}

} /* namespace Eigen */

namespace planning2d {

const extern double PI;  //! Global variable pi
const extern double TWOPI; //! Global variable for 2.*pi for speedup
const extern double LOG_TWOPI; //! Global variable for log(2.*pi) for speedup

typedef int64_t Id;
#define ID_INVALID std::numeric_limits<planning2d::Id>::min()

} /* namespace planning2d */
  
#endif /* PLANNER_INTERFACES_SUPPORT_HPP_ */
