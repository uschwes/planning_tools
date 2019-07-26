###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(probabilistic_planner)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  date_time
  serialization
  program_options
)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++0x)

# enable timing via cmake option -Dprobabilistic_planner_ENABLE_TIMING
# cmake -Dprobabilistic_planner_ENABLE_TIMING=1
set(${PROJECT_NAME}_ENABLE_TIMING OFF CACHE BOOL "Enable sm::timing based profiling in ${PROJECT_NAME}")
if (${PROJECT_NAME}_ENABLE_TIMING)
  message(STATUS "${PROJECT_NAME}: Timing enabled")
  add_definitions(-D${PROJECT_NAME}_ENABLE_TIMING)
endif()

#################
## Compilation ##
#################

cs_add_library(${PROJECT_NAME}
  src/ProbabilisticPlanner.cpp
  src/ProbabilisticLearner.cpp
  src/state_representation/SceneSnapshot.cpp
  src/state_representation/ContinuousScene.cpp
  src/state_representation/Trajectory.cpp
  src/state_representation/OptAgentTypeRegistry.cpp
  src/features/RawFeature.cpp
  src/features/RawFeatureSingleton.cpp
  src/features/RawFeaturePairwise.cpp
  src/features/FeatureContainer.cpp
  src/features/Features.cpp
  src/features/ErrorTermLearning.cpp
  src/sampling/Autocorrelation.cpp
  src/FeatureScaler.cpp
  src/support/DensityComputations.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)


cs_add_library(${PROJECT_NAME}-test-support
  test/Support.cpp
)
target_link_libraries(${PROJECT_NAME}-test-support
  ${PROJECT_NAME}
)


# Profile app for Jacobians
cs_add_executable(${PROJECT_NAME}-profile-jacobians test/ProfileEvaluateJacobiansExpressions.cpp)
target_link_libraries(${PROJECT_NAME}-profile-jacobians ${PROJECT_NAME} ${PROJECT_NAME}-test-support ${Boost_LIBRARIES})
cs_add_executable(${PROJECT_NAME}-profile-expressions-vs-evaluators
  test/ProfileExpressionsVsEvaluators.cpp
)
target_link_libraries(${PROJECT_NAME}-profile-expressions-vs-evaluators
  ${PROJECT_NAME}
  ${PROJECT_NAME}-test-support
)

##################
## Installation ##
##################

cs_install()

#############
## Exports ##
#############

cs_export(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

#################
## Executables ##
#################

cs_add_executable(${PROJECT_NAME}-generatePropertyTree src/generatePropertyTreeXml.cpp)

target_link_libraries(${PROJECT_NAME}-generatePropertyTree 
  ${PROJECT_NAME}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/TestSupport.cpp
  test/TestSerialization.cpp
  test/TestTrajectory.cpp
  test/TestOptAgent.cpp
  test/TestSceneSnapshot.cpp
  test/TestContinuousScene.cpp
  test/TestOptimization.cpp
  test/TestFeatures.cpp
  test/TestFeatureScaler.cpp
  test/TestSplineFit.cpp
  test/testPlanner.cpp
  test/TestLearning.cpp
  test/Profiling.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    ${PROJECT_NAME}-test-support
    ${GTEST_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
  )
endif()
