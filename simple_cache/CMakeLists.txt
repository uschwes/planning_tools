###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(simple_cache)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED system serialization)

catkin_simple()

add_definitions(-std=c++11)

##############
## Building ##
##############

# cs_add_library(${PROJECT_NAME}
#   src/Simple.cpp
#   src/PropertyTreeValueStore.cpp
# )

##################
## Installation ##
##################

cs_install()

#############
## Exports ##
#############

cs_export(INCLUDE_DIRS ${Boost_INCLUDE_DIRS} LIBRARIES ${Boost_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/SimpleCacheTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
