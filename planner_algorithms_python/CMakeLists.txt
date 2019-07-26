###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(planner_algorithms_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

# Set up the python exports.
set(PY_PROJECT_NAME ${PROJECT_NAME})
set(PY_PACKAGE_DIR python/${PROJECT_NAME})

##############
## Building ##
##############

add_python_export_library(${PROJECT_NAME} python/planner_algorithms
  src/python_module.cpp
  src/DijkstraPy.cpp
  src/MapChangeDetectionPy.cpp
  src/VoronoiPy.cpp
  src/DistanceTransformPy.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Exports ##
#############
cs_export(LIBRARIES ${PROJECT_NAME})

##################
## Installation ##
##################
cs_install()

#############
## Testing ##
#############

catkin_add_nosetests( 
  test/Test.py
  DEPENDENCIES ${PROJECT_NAME}
)