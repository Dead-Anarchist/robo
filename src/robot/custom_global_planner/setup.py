#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['custom_global_planner'],
    package_dir={'': 'src'}
)
setup(**setup_args)
