/*
 * Profiling.cpp
 *
 *  Created on: 11.08.2015
 *      Author: Ulrich Schwesinger
 */

// standard includes
#include <iostream>

// gtest
#include <gtest/gtest.h>

// Schweizer Messer
#include <sm/timing/Timer.hpp>

#ifdef probabilistic_planner_ENABLE_TIMING
TEST(probabilistic_planner_TESTSUITE, Profiling) {
    sm::timing::Timing::print(std::cout, sm::timing::SORT_BY_TOTAL);
}
#endif
