from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['nodes/tactile_test_sensors.py'],
    package_dir={'': 'nodes'})

setup(**setup_args)
