#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['segbot_gui'],
    package_dir={'': 'src'},
    scripts={'scripts/question_dialog_plugin'},
)

setup(**d)
