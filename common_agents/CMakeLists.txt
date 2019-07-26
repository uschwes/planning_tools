###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(common_agents)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  date_time
  serialization
)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++0x)

###############
## Libraries ##
###############

cs_add_library(${PROJECT_NAME}
  src/DifferentialDriveState.cpp
  src/DifferentialDriveSystemInput.cpp
  src/DifferentialDriveAgent.cpp
  src/HolonomicState.cpp
  src/HolonomicSystemInput.cpp
  src/HolonomicAgent.cpp
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

cs_export()

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test
 test/test_main.cpp
 test/UnitTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
 target_link_libraries(${PROJECT_NAME}-test 
   ${PROJECT_NAME}
   ${GTEST_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
 )
endif()
