###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(planner_algorithms)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
)
find_package(Threads REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
)

# enable timing via cmake option -Dplanner_algorithms_ENABLE_TIMING
set(${PROJECT_NAME}_ENABLE_TIMING OFF CACHE BOOL "Enable sm::timing based profiling in ${PROJECT_NAME}")
if (${PROJECT_NAME}_ENABLE_TIMING)
  message(STATUS "${PROJECT_NAME}: Timing enabled")
  add_definitions(-D${PROJECT_NAME}_ENABLE_TIMING)
endif()

##############
## Building ##
##############

cs_add_library(${PROJECT_NAME}
  src/MapChangeDetection.cpp
  src/Voronoi.cpp
  src/DistanceTransform.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)


##################
## Installation ##
##################

cs_install()

#############
## Exports ##
#############

# Append boost libraries and include directories so they get exported to 
# downstream packages automatically

cs_export(
  INCLUDE_DIRS ${Boost_INCLUDE_DIRS} 
  LIBRARIES ${Boost_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/OpenCvConversionsTest.cpp
    test/DijkstraTest.cpp
    test/DijkstraProfiling.cpp
    test/MapChangeDetectionTest.cpp
    test/VoronoiTest.cpp
    test/DistanceTransformTest.cpp
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test 
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
      ${GTEST_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
    )
  endif()
endif()
