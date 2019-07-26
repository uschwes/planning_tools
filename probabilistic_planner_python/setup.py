## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['probabilistic_planner', 'probabilistic_planner.util'],
    package_dir={'':'python'},
    scripts=['bin/ToyPlanner.py', 'bin/ToyLearner.py'])

setup(**setup_args)