###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(planner_interfaces_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

# Set up the python exports.
set(PY_PROJECT_NAME ${PROJECT_NAME})
set(PY_PACKAGE_DIR python/${PROJECT_NAME})

##############
## Building ##
##############

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/python_module.cpp
  src/NumpyEigen21DontAlign.cpp
  src/ReturnCodePy.cpp
  src/TimePy.cpp
  src/StampedTypePy.cpp
  src/Position2dPy.cpp
  src/Pose2dPy.cpp
  src/StatePy.cpp
  src/SystemInputPy.cpp
  src/AgentPy.cpp
  src/ShapePy.cpp
  src/OccupancyGridPy.cpp
  src/PlannerInterfacePy.cpp
  src/TargetPlannerInterfacePy.cpp
  src/TrajectoryPy.cpp
  src/ArchivePy.cpp
  src/MathSupportPy.cpp
)
list(APPEND ${PROJECT_NAME}_LIBRARIES ${PY_PROJECT_NAME})
target_link_libraries(${PY_PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Exports ##
#############
cs_export()

##################
## Installation ##
##################
cs_install()

#############
## Testing ##
#############

catkin_add_nosetests( 
  test/Test.py
  DEPENDENCIES ${PY_PROJECT_NAME}
)