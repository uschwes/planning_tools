/*
 * Support.hpp
 *
 *  Created on: 23.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_HPP_

#include <sm/timing/Timer.hpp>

#include <planner_interfaces/Support.hpp>

namespace prob_planner {

#ifdef probabilistic_planner_ENABLE_TIMING
  typedef sm::timing::Timer Timer;
#else
  typedef sm::timing::DummyTimer Timer;
#endif

static constexpr planning2d::Id ID_EGO = 0;

}

#endif /* INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_HPP_ */
