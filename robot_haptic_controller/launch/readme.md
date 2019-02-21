lwr_robot_ravin: run my program loading parameters for ravin's experiments
lwr_robot_ravin_rtk: same but run the program in RobotSimulator, cannot control the robot (prbably for debug)
lwr_robot_ravin_no_interface_TEST: 


## New launcher: hand_only_demo.launch


# Exploration_ressources
'This is used to do the exploration+grasping in simulation'


# New needed commands to run planning in parrallel:
# Run the planner server (move_group)
roslaunch lwr_moveit_config demo.launch (need to disable Rviz for this)
# Run the planner client (move group interface in python)
rosrun lwr_moveit_config moveit_tuto.py /joint_states:=/allegro_kuka/joint_states
# Run my code

# How to run voice recognition:
(make sure that ros-indigo-audio-common is installed)
roslaunch pocketsphinx allegro_control.launch 

# TODO: run a launcher to run the hand only (no robot involved).
# But with Ravin's open/close versions ?? 
# Or my previous version to close on an object, before I used it with ravin ..
# I need to look again at the different launch files, the parameters and the code 