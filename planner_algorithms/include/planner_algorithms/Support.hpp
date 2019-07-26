#ifndef PLANNER_ALGORITHMS_SUPPORT_HPP_
#define PLANNER_ALGORITHMS_SUPPORT_HPP_

// Schweizer Messer includes
#include <sm/timing/Timer.hpp>

namespace planning2d
{
namespace algorithms
{

#ifdef planner_algorithms_ENABLE_TIMING
  typedef sm::timing::Timer Timer;
#else
  typedef sm::timing::DummyTimer Timer;
#endif

} /* namespace planning2d */
} /* namespace algorithms */
  
#endif /* PLANNER_ALGORITHMS_SUPPORT_HPP_ */
