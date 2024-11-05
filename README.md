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

roslaunch carter_2dnav_zone_GA multiple_robot_carter_navigation.launch env_name:=LEfloor
    
12. In another terminal run:

roslaunch isaac_sim_zone_GA isaac_sim_zone.launch

13. In another terminal run:

./clear_costmap.sh


Current Parameters:

V = 236.2 (robot velocity) (foot/min)

tl = 0.042 (load time) (min)

tu = 0.042

T = 8 (length of work day) (hours)

lt = 120 (expexted lost time due to charging) (min)


adj_dist = 243.64 (feet) distance for another workstation to be considered adjacent

gen_size = 100 # of generations 

pop_size = 60 population size

mutation_rate = 0.45

selection_size = 10

Ca = 0.5 weights for shortest job first with aging

Cd = 10 

IM_freq = 1 (min) frequency to sample the current zone balence in "intensive monitoring mode"

NM_freq = 3 (min) frequency to sample the current zone balence in "non-intensive monitoring mode"

t1 = 5 (min) time to switch to "non-intensive monitoring mode"

t2 = 15 (min) time to start zone repair 

SV_pTol = 0.2 highest SV_p value accepted before an imbalence is declared
