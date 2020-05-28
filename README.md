# robot_haptic_controller
Forked from Nisommer, debugged to run on ros-kinetic

This is the package used in our joint work published in Nature Machine Intelligence journal:


https://www.nature.com/articles/s42256-019-0093-5



This package was written by the second author. My contribution to the project was to adapt this package to implement it on real robots (KUKA arm and Allegro hand). 

# Brief description of project:

Accompanying Youtube video:

https://www.youtube.com/watch?v=YzddAsGUJ84

This is a project on shared control of prosthesis hand using muscle activity of handicapped subjects. Neureal networks were implement to decode EMG (electomyocardiogram) signals for inferring the intentions of subject. In conjunction with that, information from tacticle sensors placed on robotic hand was used to improve the grasp quality when controlling the robotic hand. The package here is only a controller of the robotic hand, which tries to increase the area of contact between the object and robotic fingers once user's intention to grasp is confirmed.


# Requirements

Basic reuirements:
1. ROS kinetic running on Linux platform
2. Point Cloud Library, TEKSCAN wrapper (available on request).

Pre-requisites for running on real robots:
1. KUKA IIWA robotic arm with corresponding driver and controllers installed
2. Allegro hand with corresponding driver and controllers installed (Torque control mode required)
3. EMG sensors and decoder (alternatively input about grasp intention can be given manually with keyboard)

Pre-requisites for running on simulator:
1. Gazebo
2. Controllers for the above robots integrated with gazebo plugins
3. Models of the above robots (urdf files) - available on request

# Commands for running

real robot:
roslaunch allegro_hand_gazebo allegro_hand_objects.launch

simulation
roslaunch robot_haptic_controller hand_only_demo_simulation_objects.launch


