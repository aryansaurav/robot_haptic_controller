# TODO LIST

- Do proper changes from robot control modes: EASY WAY TO SWITCH !!!!!


TODO See how I can move the contacts forces code in different functions (but need still to test that contacts works !)
	- Contact force controller
	- Virtual contact force controller
	- GetVirtualDirection ...
	- VCPOINT_with_normal : VCPOINT

	--> Put all that in another c++ file.
	--> Communicate with functions.

	START WITH SOME CODE THAT I CAN EASILY TEST


TEST:  COMPILATION WHILE REMOVING robot UPDATE ???



A) Dynamic parameters: 
	TEST changing the impedance ...
B) Check Smash:
	-check Felix's example
	--> hard !

-- 


HAND TORQUE CONTROL: If it does not receive a torque for some time, switch to position control with current position.


#USEFUL
locate --basename .mp4 .mkv .wmv .flv .webm .mov .avi
find -iregex '.*\.\(mp4\|avi\|mkv\|MP4\)$'
find ./ -type f \( -iname \*.mp4 -o -iname \*.avi \)

#How to switch from hand1 to hand2 (modified versions):
b_thumb_from_opt
sim_robot_new_...
allegro_kuka_any -> make modifs
