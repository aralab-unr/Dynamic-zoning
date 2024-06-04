# Dynamic-zoning
Russell Keith code with simulated annealing and genetic algorithm

INSTALLATION AND RUN INSTRUCTIONS

1. Install Omniverse and download Isaac Sim, instructions for which can be found here:
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html
 
2. Install ROS Noetic, and create a workspace from the Isaac ROS Workspace Repository. Then enable the ROS bridge extension in Isaac sim. Instructions can be found here:
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros

3. Install the ROS Navigation stack:
sudo apt-get install ros-$ROS_DISTRO-navigation
    
4. Replace the navigation folder in <path to ROS  workspace>/src with the folder of the same name found above.
 
5. Download warehouse_build.usd and place in accessible location.
 
6. Open Isaac Sim → file → open → navigate to file →  warehouse_build.usd

7. Press the play button in the tool bar to the left
    
8. In a terminal run:
roslaunch carter_2dnav multiple_robot_carter_navigation.launch env_name:=warehouse_build
    
9. In another terminal run:
roslaunch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch
