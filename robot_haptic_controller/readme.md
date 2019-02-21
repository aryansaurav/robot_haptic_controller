My robot controller as a RobotToolKit Module


- Continuous active compliance with EMG.
* needed: bridge to get desired joint positions from EMG decoding
-> simulate with a GUI for now:
#Run the joint state publisher with GUI ...
#rosrun joint_state_publisher joint_state_publisher _use_gui:=true /robot_description:=/allegroHand_1/robot_description
# with modified output topic ...
rosrun joint_state_publisher joint_state_publisher _use_gui:=1 robot_description:=/allegroHand_1/robot_description joint_states:=desired_hand_state


#Some fake joints:
rosbag play gloveData_2017-04-06-16-28-04.bag /allegroHand_1/joint_states:=/desired_hand_state 


a)
roslaunch epfl_allegro_launchers epfl_right.launch

b)
python load_msg_v2.py  



## OLD SIMU (with full Kuka) Simulation: compliant grasping, seems to run well with:
roslaunch allegro_kuka_gazebo allegro_kuka_any.launch file:=handle
# then I need to adapt allegro_kuka_handle.world to choose which object ...

###### SIMULATION

# Run the simulated world with objects (this uses the spacemouse to move objects around):
roslaunch allegro_hand_gazebo allegro_hand_objects.launch


# Run the controller for the simulation
roslaunch robot_haptic_controller hand_only_demo_simulation_objects.launch 