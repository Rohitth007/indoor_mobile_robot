from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=[],
     package_dir={'': 'src'},
     install_requires = ['numpy']
)

setup(**setup_args)
