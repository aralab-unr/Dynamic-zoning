from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["goal_generators","zone_SA", "obstacle_map"], package_dir={"": "isaac_ros_navigation_goal_SA"}
)
setup(**d)
