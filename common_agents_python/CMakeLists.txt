###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(common_agents_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# TODO: this should get exported to downstream packages by planner_interfaces
find_package(Boost REQUIRED COMPONENTS
  date_time
)

catkin_python_setup()

# Set up the python exports.
set(PY_PROJECT_NAME ${PROJECT_NAME})
set(PY_PACKAGE_DIR python/${PROJECT_NAME})

##############
## Building ##
##############

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/HolonomicStatePy.cpp
  src/HolonomicSystemInputPy.cpp
  src/HolonomicAgentPy.cpp
  src/DifferentialDriveStatePy.cpp
  src/DifferentialDriveSystemInputPy.cpp
  src/DifferentialDriveAgentPy.cpp
  src/TypeRegistrationPy.cpp
  src/python_module.cpp
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