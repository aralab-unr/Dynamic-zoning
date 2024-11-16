from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["goal_generators","assets", "obstacle_map"], package_dir={"": "scripts"}
)
setup(**d)
