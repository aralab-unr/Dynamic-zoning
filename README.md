# Dynamic-zoning
Russell Keith code with simulated annealing and genetic algorithm

INSTALLATION AND RUN INSTRUCTIONS

1. Install Omniverse and download Isaac Sim, instructions for which can be found here: 
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html
 
2. Install ROS Noetic, and create a workspace from the Isaac ROS Workspace Repository. Then enable the ROS bridge extension in Isaac sim. Instructions can be found here: 
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros

3. Install the ROS Navigation stack: 
   sudo apt-get install ros-$ROS_DISTRO-navigation
    
5. The workspace will have a bunch of turtorials located inside the /src folder. Replace the navigation folder in <path to ROS  workspace>/src with the folder of the same name found above.

6. Run:
   catkin_make
   Then re-source each terminal
   source ~/.bashrc
 
8. Download warehouse_build.usd and place in accessible location.
 
9. Open Isaac Sim → file → open → navigate to file →  warehouse_build.usd

10. Press the play button in the tool bar to the left
    
11. In a terminal run:
    
 roslaunch carter_2dnav_ddzone multiple_robot_carter_navigation.launch env_name:=LEfloor

13. In another terminal run:

 roslaunch ddzoning sim.launch

14. In another terminal run:
 
 ./clear_costmap.sh


Current parameters:

V = 236.2 (foot/min) robot velocity

tl = 0.042 (min) load time

tu = 0.042 (min) unload time

adj_dist = 300.00 (feet) distance for another workstation to be considered adjacent

Ca = 0.5 weights for shortest job first with aging

Cd = 10 

L_TOL = 18 (min) maximum variation in standard deviation until a imbalence is declared

T_LT = 15 (min) amount of time till zone repair

T_AC = 2 (min) fequency to run weighted average consensus

E_t = 2 # of episodes

I_t = 10 # number of iterations

