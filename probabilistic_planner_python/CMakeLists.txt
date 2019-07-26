###########
## Setup ##
###########

cmake_minimum_required(VERSION 2.8.3)
project(probabilistic_planner_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

##############
## Building ##
##############
add_python_export_library(${PROJECT_NAME} python/probabilistic_planner
  src/ContinuousScenePy.cpp
  src/SceneSnapshotPy.cpp
  src/TrajectoryPy.cpp
  src/OptAgentPy.cpp
  src/FeatureContainerPy.cpp
  src/RawFeaturePy.cpp
  src/RawFeatureSingletonPy.cpp
  src/RawFeaturePairwisePy.cpp
  src/FeatureImplementationsPy.cpp
  src/FeatureScalerPy.cpp
  src/ProbabilisticLearnerPy.cpp
  src/AddGridErrorTerm2dPy.cpp
  src/AutocorrelationPy.cpp
  src/DensityComputationsPy.cpp
  src/python_module.cpp
)

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
  DEPENDENCIES ${PROJECT_NAME}
)
