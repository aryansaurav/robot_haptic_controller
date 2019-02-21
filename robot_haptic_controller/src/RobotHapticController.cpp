/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "RobotHapticController.h"
#include <cmath>


#include <iostream>
#include <string>
#include <random>

#define ENABLE_FINGER_0 true
#define ENABLE_FINGER_1 true
#define ENABLE_FINGER_2 true
#define ENABLE_FINGER_3 true


RobotHapticControllerClass::RobotHapticControllerClass()
    : RobotInterface() {}

RobotHapticControllerClass::~RobotHapticControllerClass()
{}

RobotInterface::Status RobotHapticControllerClass::RobotInit() {
    // Initialize the steps counter, important to have at the beginning
    i_counter = 0;


    /**  =================================
    * =      Create Boost Thread      =
    * ================================= */
    m_full_contact_jacobian.Resize(0, 0);
    b_new_Jacobian   = false;
    b_new_Jacobian_c = false;

    // Start the boost thread
    bt_thread1_c = thread( std::bind(&RobotHapticControllerClass::NullSpaceThread_c, this) );


    /**  ==================================
    * =  Create ros topic connections  =
    * ================================== */

    //    this->rosnode_ = mRobot->InitializeROS("nico_robot_control");
    int   argc = 0;
    char *argv[1];
    ros::init(argc, argv, "nico_robot_control", ros::init_options::NoSigintHandler);
    this->rosnode_ = new ros::NodeHandle("nico_robot_control");


    //    this->rosnode_ = mRobot->InitializeROS("nico_robot_control");

    myPublisher = rosnode_->advertise<my_msgs::customMsg1>("/myPubDefault", 1); // advertise something else ....


    pub_task_state = rosnode_->advertise<std_msgs::String>("/task_state", 1);


    // TODO: UNused for now ...
    topic_name_ravin_reached = "/reached";
    this->pub_ravin_reached  = this->rosnode_->advertise<std_msgs::Bool>(topic_name_ravin_reached, 1);

    // Object impedance for gazebo (object that is held)
    topic_name_gazebo_object_impedance = "/gazebo/object_impedance";
    this->pub_gazebo_object_impedance  = this->rosnode_->advertise<std_msgs::Float32MultiArray>(topic_name_gazebo_object_impedance, 1);


    // Publish the state of the hand (for debug / visualization purpose)
    this->pub_hand_state  = this->rosnode_->advertise<sensor_msgs::JointState>("/allegroHand/joint_states", 1);
    this->pub_robot_state = this->rosnode_->advertise<sensor_msgs::JointState>("/allegro_kuka/joint_states", 1);
    this->pub_robot_state2 = this->rosnode_->advertise<sensor_msgs::JointState>("/allegro_kuka/joint_states_debug", 1);

    this->pub_info_ravin   = this->rosnode_->advertise<my_msgs::info_ravin>("/info_for_ravin", 1);
    this->pub_string_debug = this->rosnode_->advertise<std_msgs::String>("/Nicolas/debug", 1);


    // Subscribe to ravin's mode message (desired patches for contact)
    topic_name_ravin_mode = "/mode";
    this->sub_ravin_mode  = this->rosnode_->subscribe<my_msgs::Mode>(this->topic_name_ravin_mode, 1, &RobotHapticControllerClass::RavinCallbackMode, this);


    // Subscribe to Ravin's task message (defines a cartesian rotation right now)
    this->sub_ravin_task = this->rosnode_->subscribe<my_msgs::Task>("/task", 10, &RobotHapticControllerClass::RavinCallbackTaskCartesianRotation, this);


    // Subscribe to Ravin's empty message (to trigger close)
    this->sub_ravin_close = this->rosnode_->subscribe<std_msgs::Empty>("/close", 1, &RobotHapticControllerClass::RavinCallbackEmptyClose, this);

    // Subscribe to Ravin's empty message (to trigger tightening)
    this->sub_ravin_tighten = this->rosnode_->subscribe<std_msgs::Empty>("/tighten", 1, &RobotHapticControllerClass::RavinCallbackEmptyTighten, this);

    topic_name_ravin_trajectory = "/trajectory";
    this->sub_ravin_trajectory  = this->rosnode_->subscribe<sensor_msgs::JointState>(this->topic_name_ravin_trajectory, 1, &RobotHapticControllerClass::RavinCallbackTrajectory, this);


    topic_name_audio_recognition = "/nl_command_parsed";
    sub_audio_recognition        = this->rosnode_->subscribe<std_msgs::String>(this->topic_name_audio_recognition,
                                                                               1,
                                                                               boost::bind(&RobotHapticControllerClass::AudioRecognitionCallback, this, _1)
                                                                               );


    this->sub_string_command = this->rosnode_->subscribe<std_msgs::String>("/haptic_controller/input", 10, &RobotHapticControllerClass::GetCommandString, this);

    this->sub_grasping_perturbation = this->rosnode_->subscribe<std_msgs::Float32MultiArray>("/grasping_perturbation", 1, &RobotHapticControllerClass::CallbackGraspingPerturbation, this);

    this->sub_pyconsole = this->rosnode_->subscribe<std_msgs::String>("/py_console_talker", 1, &RobotHapticControllerClass::RespondToConsoleCommandByTopic, this);

    this->pub_pyconsole = this->rosnode_->advertise<std_msgs::String>("/py_console_listener", 1);


    /** -------------------
    *      Space Nav
    * -------------------*/
    topic_name_spacenav_twist = "/spacenav/twist";
    topic_name_spacenav_joy   = "/spacenav/joy";

    this->twist_msg_.linear.x  = 0;
    this->twist_msg_.linear.y  = 0;
    this->twist_msg_.linear.z  = 0;
    this->twist_msg_.angular.x = 0;
    this->twist_msg_.angular.y = 0;
    this->twist_msg_.angular.z = 0;

    this->joy_msg_.buttons.clear();
    stdvec_prev_joy_buttons.clear();

    this->sub_space_nav_twist = this->rosnode_->subscribe(topic_name_spacenav_twist, 1, &RobotHapticControllerClass::UpdateSpaceNavDataTwist, this);
    this->sub_space_nav_joy   = this->rosnode_->subscribe(topic_name_spacenav_joy, 1, &RobotHapticControllerClass::UpdateSpaceNavDataJoy, this);

    this->sub_moveit_planned_trajectory = this->rosnode_->subscribe("/move_group/planned_trajectory",1,&RobotHapticControllerClass::RobotTrajectoryCallBack, this);
    this->sub_des_hand_config = this->rosnode_->subscribe("/hand_coupled_ds/DS2_desired_hand_config",1,&RobotHapticControllerClass::HandConfigCallback, this);
    //    this->sub_des_hand_config = this->rosnode_->subscribe("/desired_hand_state",1,&RobotHapticControllerClass::HandConfigCallback, this);
    this->sub_des_hand_config = this->rosnode_->subscribe("/desired_hand_state",1,&RobotHapticControllerClass::HandStateCallback, this);

    br = new tf2_ros::TransformBroadcaster();


    /** --------------------------------------------------------------
    *      Gazebo Sim: publish torques, subscribe joint positions
    * --------------------------------------------------------------*/

    // Init the publisher
    this->pub_array              = this->rosnode_->advertise<std_msgs::Float32MultiArray>("/Nicolas/array", 1);
    this->pub_torques            = this->rosnode_->advertise<std_msgs::Float32MultiArray>("/lwr/input_torques/", 1);
    //    this->pub_torques            = this->rosnode_->advertise<std_msgs::Float32MultiArray>("compliant_exploration_torques", 1);
    this->pub_pcl                = this->rosnode_->advertise<PointCloud>("/pcl/", 1);
    this->pub_cart_target_gazebo = this->rosnode_->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    this->pub_tactile_pressures  = this->rosnode_->advertise<std_msgs::Float32MultiArray>("/tactile_pressures", 1);


    // Visualization (riz)
    pub_marker_array = this->rosnode_->advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);


    // / Allegro Hand

    // Desired hand state (efforts) publisher (for real allegro hand)
    this->pub_hand_state_des_torque = this->rosnode_->advertise<sensor_msgs::JointState>("/allegroHand_1/torque_cmd", 1);


    // Hand state subscriber (from real allegro hand)
    topic_name_hand_joint_state = "/allegroHand_1/joint_states";
    this->sub_hand_state        = this->rosnode_->subscribe( topic_name_hand_joint_state, 3, &RobotHapticControllerClass::CallbackAllegroJointState, this, ros::TransportHints().tcpNoDelay() );


    // Get joint position from Gazebo
    topic_name_joint_state2 = "/lwr/output_positions2/";
    this->sub_joint_state2  = this->rosnode_->subscribe( this->topic_name_joint_state2, 1, &RobotHapticControllerClass::UpdateGazeboJointState, this, ros::TransportHints().tcpNoDelay() );


    // Get contacts from Gazebo
    topic_name_contact_state = "/link_collision_sensor";
    this->sub_contact_state  = this->rosnode_->subscribe( this->topic_name_contact_state, 1, &RobotHapticControllerClass::UpdateSimContactState, this, ros::TransportHints().tcpNoDelay() );


    task_rotation  = new TaskRotation();
    task_hammering = new TaskHammering();


    /**   =================================
     * =         Get parameters        =
     * ================================= */


    // Dyn parameters callback
    // Init the server
    dyn_reconf_server          = new dynamic_reconfigure::Server<my_msgs::my_dyn_paramsConfig>();
    dyn_reconf_callback_binded = boost::bind(&RobotHapticControllerClass::callback, this, _1, _2);
    dyn_reconf_server->setCallback(dyn_reconf_callback_binded);


    cerr << "this->rosnode_->getNamespace(): " << this->rosnode_->getNamespace() << endl;
    getParamSafeHandle("b_handonly_nullspace",                     b_handonly_nullspace);
    getParamSafeHandle("b_simulate_hand_only",                     b_simulate_hand_only);

    //  b_handonly_nullspace_next = b_handonly_nullspace;
    getParamSafeHandle("b_ravin_parrallel_closing",                b_ravin_parrallel_closing);
    getParamSafeHandle("b_ravin_use_orientation_for_vc",           b_ravin_use_orientation_for_vc);
    getParamSafeHandle("b_ravin_allow_tighten_switch",             b_ravin_allow_tighten_switch);
    getParamSafeHandle("b_ravin_use_fake_mode_data",               b_ravin_use_fake_mode_data);
    getParamSafeHandle("b_ravin_auto_close_switch",                b_ravin_auto_close_switch);
    getParamSafeHandle("b_ravin_bypass_hand_error_check_reach",    b_ravin_bypass_hand_error_check_reach);
    getParamSafeHandle("d_ravin_min_time_for_reach_finished",      d_ravin_min_time_for_reach_finished);
    getParamSafeHandle("d_ravin_patch_pressure_mutliplier",        d_ravin_patch_pressure_mutliplier);
    getParamSafeHandle("d_ravin_hand_error_norm_threshold_reach",  d_ravin_hand_error_norm_threshold_reach);
    getParamSafeHandle("d_ravin_hand_velocity_threshold_reach",    d_ravin_hand_velocity_threshold_reach);
    getParamSafeHandle("d_ravin_time_auto_mini_tighten_switch",    d_ravin_time_auto_mini_tighten_switch);
    getParamSafeHandle("d_ravin_max_time_unscrewing",              d_ravin_max_time_unscrewing);
    getParamSafeHandle("d_ravin_joint_limit_threshold_unscrewing", d_ravin_joint_limit_threshold_unscrewing);
    getParamSafeHandle("b_protect_undesired_contacts",             b_protect_undesired_contacts);
    getParamSafeHandle("b_use_filtered_cartesian_target",          b_use_filtered_cartesian_target);
    getParamSafeHandle("i_pd_grasp",                               i_pd_grasp);

    getParamSafeHandle("d_vc_cart_kp",                             d_vc_cart_kp);
    getParamSafeHandle("d_vc_cart_kd",                             d_vc_cart_kd);
    getParamSafeHandle("d_hand_kp_multiplier",                     d_hand_kp_multiplier);
    getParamSafeHandle("d_hand_kd_multiplier",                     d_hand_kd_multiplier);
    getParamSafeHandle("d_vc_cart_max_gap",                        d_vc_cart_max_gap);

    getParamSafeHandle("d_temp_1",                                 d_temp_1);
    getParamSafeHandle("d_temp_2",                                 d_temp_2);
    getParamSafeHandle("d_temp_3",                                 d_temp_3);
    getParamSafeHandle("d_temp_4",                                 d_temp_4);
    getParamSafeHandle("d_temp_5",                                 d_temp_5);
    getParamSafeHandle("d_temp_6",                                 d_temp_6);
    getParamSafeHandle("d_joint_close_velocity",                   d_joint_close_velocity);
    getParamSafeHandle("d_temp_8",                                 d_temp_8);
    getParamSafeHandle("d_temp_9",                                 d_temp_9);
    getParamSafeHandle("d_temp_10",                                d_temp_10);

    getParamSafeHandle("i_temp_1",                                 i_temp_1);
    getParamSafeHandle("i_temp_2",                                 i_temp_2);
    getParamSafeHandle("i_temp_3",                                 i_temp_3);

    getParamSafeHandle("b_display_time",                           b_display_time);
    getParamSafeHandle("b_publish_virtual_contact_frames_markers", b_publish_virtual_contact_frames_markers);

    getParamSafeHandle("b_no_projection",                          b_no_projection);
    getParamSafeHandle("b_pd_in_nullspace",                        b_finger_pd_in_nullspace);
    getParamSafeHandle("b_use_qp",                        b_use_qp);
    getParamSafeHandle("b_disable_index",                        b_disable_index);


    getParamSafeHandle("d_task_weight_vc",                         d_task_weight_vc);
    getParamSafeHandle("d_task_weight_space_mouse",                d_task_weight_space_mouse);
    getParamSafeHandle("d_task_weight_joint_centering",            d_task_weight_joint_centering);
    getParamSafeHandle("d_task_weight_joint_position",             d_task_weight_joint_position);

    getParamSafeHandle("d_task_weight_joint_limit",                d_task_weight_joint_limit);
    getParamSafeHandle("d_min_jointlimit_distance_percent_arm",    d_min_jointlimit_distance_percent_arm);
    getParamSafeHandle("d_max_torque_arm_centering",               d_max_torque_arm_centering);

    getParamSafeHandle("d_jsim_regularization",                     d_jsim_regularization);
    getParamSafeHandle("b_no_thumb_from_opt",                     b_no_thumb_from_opt);
    getParamSafeHandle("d_target_reached_threshold",                     d_target_reached_threshold);
    getParamSafeHandle("d_angle_sum_max_limit",                     d_angle_sum_max_limit);
    getParamSafeHandle("b_enable_spmouse_buttons",                     b_enable_spmouse_buttons);

    getParamSafeHandle("d_ns_torques_multiplier",                     d_ns_torques_multiplier);
    getParamSafeHandle("d_grav_comp_multiplier",                     d_grav_comp_multiplier);



    b_use_gazebo_contact_data = true; // should not be hardcoded

    // / Mapping from Gazebo names to local names
    stdmap_link_name_from_gazebo["lwr_arm_1_link"] = "DOF_0";
    stdmap_link_name_from_gazebo["lwr_arm_2_link"] = "DOF_1";
    stdmap_link_name_from_gazebo["lwr_arm_3_link"] = "DOF_2";
    stdmap_link_name_from_gazebo["lwr_arm_4_link"] = "DOF_3";
    stdmap_link_name_from_gazebo["lwr_arm_5_link"] = "DOF_4";
    stdmap_link_name_from_gazebo["lwr_arm_6_link"] = "DOF_5";
    stdmap_link_name_from_gazebo["lwr_arm_7_link"] = "DOF_6";


    stdmap_link_name_from_gazebo["ahand/thumb/dof0_link"] = "HandF4Dof0";
    stdmap_link_name_from_gazebo["ahand/thumb/dof1_link"] = "HandF4Dof1";
    stdmap_link_name_from_gazebo["ahand/thumb/dof2_link"] = "HandF4Dof2";
    stdmap_link_name_from_gazebo["ahand/thumb/dof3_link"] = "HandF4Dof3";

    stdmap_link_name_from_gazebo["ahand/finger_1/dof0_link"] = "HandF1Dof0";
    stdmap_link_name_from_gazebo["ahand/finger_1/dof1_link"] = "HandF1Dof1";
    stdmap_link_name_from_gazebo["ahand/finger_1/dof2_link"] = "HandF1Dof2";
    stdmap_link_name_from_gazebo["ahand/finger_1/dof3_link"] = "HandF1Dof3";

    stdmap_link_name_from_gazebo["ahand/finger_2/dof0_link"] = "HandF2Dof0";
    stdmap_link_name_from_gazebo["ahand/finger_2/dof1_link"] = "HandF2Dof1";
    stdmap_link_name_from_gazebo["ahand/finger_2/dof2_link"] = "HandF2Dof2";
    stdmap_link_name_from_gazebo["ahand/finger_2/dof3_link"] = "HandF2Dof3";

    stdmap_link_name_from_gazebo["ahand/finger_3/dof0_link"] = "HandF3Dof0";
    stdmap_link_name_from_gazebo["ahand/finger_3/dof1_link"] = "HandF3Dof1";
    stdmap_link_name_from_gazebo["ahand/finger_3/dof2_link"] = "HandF3Dof2";
    stdmap_link_name_from_gazebo["ahand/finger_3/dof3_link"] = "HandF3Dof3";


    // / List of links with their names
    stdvec_link_names_gazebo.clear();
    stdvec_link_names_gazebo.push_back("lwr_arm_1_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_2_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_3_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_4_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_5_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_6_link");
    stdvec_link_names_gazebo.push_back("lwr_arm_7_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_1/dof0_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_1/dof1_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_1/dof2_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_1/dof3_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_2/dof0_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_2/dof1_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_2/dof2_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_2/dof3_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_3/dof0_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_3/dof1_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_3/dof2_link");
    stdvec_link_names_gazebo.push_back("ahand/finger_3/dof3_link");
    stdvec_link_names_gazebo.push_back("ahand/thumb/dof0_link");
    stdvec_link_names_gazebo.push_back("ahand/thumb/dof1_link");
    stdvec_link_names_gazebo.push_back("ahand/thumb/dof2_link");
    stdvec_link_names_gazebo.push_back("ahand/thumb/dof3_link");


    // / List of desired cartesian positions
    stdvec_des_cart_positions.clear();

    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.9,0.0,0.2));  // front
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.8,-0.4,0.2)); // left front
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.6,-0.4,0.2)); // left back
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.5,-0.1,0.2)); // back
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.5,0.0,0.2));  // back towards torus
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.75,0.25,0.3));// torus entry
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.5,0.6,0.3));  // torus 2
    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.7,0.05,0.35));// top


    /// Version 2 sphere: points away from the surface
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.6, -0.1, 0.32) ); // top left
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.6, -0.3, 0.32) ); // top left

    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, -0.3, 0.32) ); // top left
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, -0.1, 0.32) ); // top left

    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.8, 0.0, 0.2) );   // front
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.8, 0.1, 0.2) );   // front
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(1.0, -0.4, 0.2) );  // front left
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.6, -0.5, 0.2) );  // left back
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.4, -0.1, 0.2) );  // back
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.5, 0.2, 0.2) );   // back right
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, 0.2, 0.2) );   // back right
    stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, 0.05, 0.35) ); // top
    i_curr_despos_i = 0;


    // / Version for the Torus

    //    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.0,0.9,0.2));  // down
    //    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.25,0.9,0.5));  // left
    //    stdvec_des_cart_positions.push_back(Eigen::Vector3d(-0.05,0.9,0.65));  // top

    //    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.6,-0.5,0.2)); // left back
    //    stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.4,-0.1,0.2)); // back


    //    // Version A for plate locating
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.5,0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.5,-0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.7,-0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.7,0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.8,0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.8,-0.2,0.15));
    //        stdvec_des_cart_positions.push_back(Eigen::Vector3d(0.7,-0.2,0.15));

    // Version B for plate locating
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.5, 0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.5, -0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.6, 0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.6, -0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, 0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.7, -0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.8, 0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.8, -0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.9, 0.2, 0.15) );
    //  stdvec_des_cart_positions.push_back( Eigen::Vector3d(0.9, -0.2, 0.15) );

    e_final_pos_exploration = stdvec_des_cart_positions[0];
    i_curr_despos_i         = 0;

    /** =================================
     * =    INIT TACTILE CONNECTION    =
     * ================================= */

    // Find a NameServer, Check Yarp Network

    yarp::os::Network yarp;
    yarp::os::Contact myContact;
    bool    bScanNeeded, bServerFound;
    // depends on the usage: if on LAN, leave namespace to /local
    //    yarp.setNameServerName("/Network2"); // Change the Nameserver

    bool b_use_yarp = false;
    getParamSafeHandle("b_use_yarp", b_use_yarp);

    if (b_use_yarp) {
        myContact = yarp.detectNameServer(TRUE, bScanNeeded, bServerFound); // Find a server with that nameserver

        //        myContact = Contact("128.178.145.25",10000);
        if (bServerFound) {
            cout << "Yarp Contact found : " << myContact.getHost() << ":" << myContact.getPort() << endl;
        } else {
            cout << "No yarp server found ..." << endl;
        }

        yarp.registerContact(myContact); // Use the found server
    }

    bool b_yarp_connection;

    if (b_use_yarp) {
        b_yarp_connection = yarp.checkNetwork();
    } else {
        b_yarp_connection = false;
    }


    if (!b_yarp_connection) {
        cout << "\n===================" << endl;
        cout << "= No Yarp Network =" << endl;
        cout << "===================" << endl;
        b_tekscan_con_success = 0;
    } else {
        // Connect to Tekscan port
        bUseTekscan           = 1;
        b_tekscan_con_success = 0;
        snprintf(mBaseName, 256, "RobotHapticController");

        if (bUseTekscan) {
            char portNameTactileOut[256], portNameTactileIn[256];

            snprintf(portNameTactileOut, 256, "/gloveout1");
            snprintf(portNameTactileIn,  256, "/%s/TekscanTactile/in", mBaseName);
            tactileTekscanPort.open(portNameTactileIn);

            if ( !yarp::os::Network::exists(portNameTactileOut) ) {
                cerr << "Error port: " << portNameTactileOut << " not found" << endl;
                b_tekscan_con_success = 0;
            } else {
                if ( !yarp::os::Network::connect(portNameTactileOut, portNameTactileIn) ) {
                    cerr << "Error: Tekscan connection did not work" << endl;
                    b_tekscan_con_success = 0;
                    exit(0);
                } else {
                    cerr << "Receiving Tekscan Tactile data OK" << endl;
                    b_tekscan_con_success = 1;
                }
            }
        }
    }


    /** =========================================
     * =    Change logger level   =
     * ========================================= */

    //    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //        ros::console::notifyLoggerLevelsChanged();
    //    }


    /** =========================================
     * =    Try to read robot tree from urdf   =
     * ========================================= */

    std::string s_robot_description;

    if ( this->rosnode_->getParam("robot_description", s_robot_description) ) {}
    else if ( this->rosnode_->getParam("/robot_description", s_robot_description) ) {}
    else {
        cerr << "could not load robot_description";
        exit(0);
    }


    if ( !kdl_parser::treeFromString(s_robot_description, my_tree) ) {
        ROS_FATAL("Failed to construct kdl tree");
        exit(0);
    } else {
        cerr << "= There are " << my_tree.getNrOfJoints() << " joints in my robot" << endl;
    }

    // Init urdf model
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(s_robot_description);
    cerr << "urdf name: " << urdf_model->getName() << endl;

    joint_limits_interface::JointLimits  limits;                                                         // Data structure
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint("ahand/finger_3/dof0_joint"); // Populate joint limits

    //  urdf_model->getLinks();

    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(urdf_joint, limits);


    //  cerr << "max pos: " << limits.max_position << endl; // it works
    //  cerr << "min pos: " << limits.min_position << endl;


    std::vector<string> stdv_jointnames;
    stdv_jointnames.clear();
    stdv_jointnames.push_back("lwr_arm_0_joint");
    stdv_jointnames.push_back("lwr_arm_1_joint");
    stdv_jointnames.push_back("lwr_arm_2_joint");
    stdv_jointnames.push_back("lwr_arm_3_joint");
    stdv_jointnames.push_back("lwr_arm_4_joint");
    stdv_jointnames.push_back("lwr_arm_5_joint");
    stdv_jointnames.push_back("lwr_arm_6_joint");
    stdv_jointnames.push_back("ahand/finger_1/dof0_joint");
    stdv_jointnames.push_back("ahand/finger_1/dof1_joint");
    stdv_jointnames.push_back("ahand/finger_1/dof2_joint");
    stdv_jointnames.push_back("ahand/finger_1/dof3_joint");
    stdv_jointnames.push_back("ahand/finger_2/dof0_joint");
    stdv_jointnames.push_back("ahand/finger_2/dof1_joint");
    stdv_jointnames.push_back("ahand/finger_2/dof2_joint");
    stdv_jointnames.push_back("ahand/finger_2/dof3_joint");
    stdv_jointnames.push_back("ahand/finger_3/dof0_joint");
    stdv_jointnames.push_back("ahand/finger_3/dof1_joint");
    stdv_jointnames.push_back("ahand/finger_3/dof2_joint");
    stdv_jointnames.push_back("ahand/finger_3/dof3_joint");
    stdv_jointnames.push_back("ahand/thumb/dof0_joint");
    stdv_jointnames.push_back("ahand/thumb/dof1_joint");
    stdv_jointnames.push_back("ahand/thumb/dof2_joint");
    stdv_jointnames.push_back("ahand/thumb/dof3_joint");

    // Fill in the limits
    for (int i = 0; i < stdv_jointnames.size(); i++) {
        joint_limits_interface::JointLimits limits_temp; // Data structure
        stdv_limits.push_back(limits_temp);

        // Debug
        cerr << "--joint: " << i <<  endl;
        cerr << "\n success: " << joint_limits_interface::getJointLimits(urdf_model->getJoint(stdv_jointnames[i]), stdv_limits[i]) << endl;
        cerr << "max pos: " << stdv_limits[i].max_position << endl; // it works
        cerr << "min pos: " << stdv_limits[i].min_position << endl;
    }


    /** ------------------------------------
     * Init chains for computing JSIM
     * ------------------------------------ */

    KDL::Vector kdlvec_gravity(0.0, 0.0, -1.0);

    // Create a chain for each fingertip
    my_tree.getChain("world", "ahand/finger_1/dof4_link", kdlchain_f0);
    my_tree.getChain("world", "ahand/finger_2/dof4_link", kdlchain_f1);
    my_tree.getChain("world", "ahand/finger_3/dof4_link", kdlchain_f2);
    my_tree.getChain("world", "ahand/thumb/dof4_link",    kdlchain_f3);
    my_tree.getChain("world", "lwr_arm_7_link",           kdlchain_endeff_7_link);

    // For getting the hand's base position, to plot markers in the hand's position ...
    // Create PALM CHAIN
    my_tree.getChain("world", "ahand/ahand_palm_link", kdlchain_hand_base);

    kdlfksolver_hand_base = new KDL::ChainFkSolverPos_recursive(kdlchain_hand_base);
    kdlfksolver_debug     = new KDL::ChainFkSolverPos_recursive(kdlchain_f1);

    // Display all segments:
    // / Add a segment to position it at the palm (approximately)
    KDL::Frame kdlframe_link_to_contact = KDL::Frame( KDL::Rotation().Identity(), KDL::Vector(0.00, 0.0, 0.12) );

    // Value for end-effector plate.
    kdlchain_endeff_7_link.addSegment( KDL::Segment(KDL::Joint(KDL::Joint::None), kdlframe_link_to_contact) );

    // / Create the solver for each chain, to compute the JSIM
    kdlcdp_f0 = new KDL::ChainDynParam(kdlchain_f0, kdlvec_gravity);
    kdlcdp_f1 = new KDL::ChainDynParam(kdlchain_f1, kdlvec_gravity);
    kdlcdp_f2 = new KDL::ChainDynParam(kdlchain_f2, kdlvec_gravity);
    kdlcdp_f3 = new KDL::ChainDynParam(kdlchain_f3, kdlvec_gravity);


    //    std::vector<RobotObstacle> r_obstacles;
    //    RobotObstacle   r_obs_temp;
    //    r_obs_temp.name = "lwr_arm_6_link_big";
    //    r_obs_temp.link_name = "lwr_arm_6_link";
    //    r_obs_temp.segment = KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation().Identity(), KDL::Vector(0.00, 0.0, 0.0) ));
    //    r_obs_temp.radius = 0.1;
    //    r_obstacles.push_back(r_obs_temp);

    //    r_obs_temp.name = "lwr_arm_5_link_base";
    //    r_obs_temp.link_name = "lwr_arm_5_link";
    //    r_obs_temp.segment = KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation().Identity(), KDL::Vector(0.00, 0.0, 0.0) ));
    //    r_obs_temp.radius = 0.1;
    //    r_obstacles.push_back(r_obs_temp);

    //    r_obs_temp.name = "lwr_arm_4_link_base";
    //    r_obs_temp.link_name = "lwr_arm_4_link";
    //    r_obs_temp.segment = KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation().Identity(), KDL::Vector(0.00, 0.0, 0.0) ));
    //    r_obs_temp.radius = 0.15;
    //    r_obstacles.push_back(r_obs_temp);

    //    std::vector
    //    for(int i=0;i<r_obstacles.size();i++){

    //    }


    /** =================================
     * =      INIT TEKSCAN SENSORS     =
     * ================================= */


    tekscanData.Resize(TEKSCAN_SIZE, false);
    tekscanDataOffset.Resize(TEKSCAN_SIZE, false);
    tekscanDataOffset.Zero();

    mTekscanDataFilterTau = 0.4;
    tekscanDataMatrix     = new MathLib::Matrix(TEKSCAN_ROW, TEKSCAN_COL);

    tekpatch_set_1 = new TekPatchSet();

    tekpatch_set_2 = new TekPatchSet();



    // Initialize some patches (they will also be "activated")

    double d_half_finger_width = d_temp_3;

    MathLib::Vector3 tempVec3;
    MathLib::Matrix3 R_gaz1_2_t, R_gaz23_2_t;

    // INDEX OF TEKSCAN FOR ALLEGRO HAND

    R_gaz23_2_t.Set(0, 0, 1, 1, 0, 0, 0, 1, 0);
    R_gaz1_2_t.Set(0, 0, 1, 0, -1, 0, 1, 0, 0);

    R_gaz1_2_t  = R_gaz1_2_t.Transpose();
    R_gaz23_2_t = R_gaz23_2_t.Transpose();

    MathLib::Matrix3 R_g_2_f;
    MathLib::Matrix3 R_t_2_f; // Represents transformation from standard tekscan representation to the final position of the patch (keep in mind that z is towards front of finger !)

    // Finger 0 OF ALLEGRO HAND
    // pinky0
    R_t_2_f.RotationZ(PI / 2.0);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_1->pinky0->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF1Dof1");
    tekpatch_set_1->pinky0->setPatchName("index1a");
//    tekpatch_set_1->thumb3->mColSize -= 2; // Force it not to take into account the last 2 columns (some bad data ...)

    // pinky1
    R_t_2_f.RotationZ(PI / 2.0);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_1->pinky1->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF1Dof1");
    tekpatch_set_1->pinky1->setPatchName("index1b");
//    tekpatch_set_1->thumb4->mColSize -= 2; // Force it not to take into account the last 2 columns (some bad data ...)

      // pinky2
    R_t_2_f.RotationZ(PI / 2.0);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_1->pinky2->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF1Dof1");
    tekpatch_set_1->pinky2->setPatchName("index1c");
//    tekpatch_set_1->thumb4->mColSize -= 2; // Force it not to take into account the last 2 columns (some bad data ...)

  // pinky3
    R_t_2_f.RotationZ(PI / 2.0);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_1->pinky3->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF1Dof1");
    tekpatch_set_1->pinky3->setPatchName("index1c");
//    tekpatch_set_1->thumb4->mColSize -= 2; // Force it not to take into account the last 2 columns (some bad data ...

    // ring1
    R_t_2_f.Identity();
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.017, d_half_finger_width);
    tekpatch_set_1->ring1->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF1Dof2");
    tekpatch_set_1->ring1->setPatchName("index2a");


    // ring2
    R_t_2_f.RotationZ(-PI / 2.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.025, d_half_finger_width);
    tekpatch_set_1->ring2->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF1Dof2");
    tekpatch_set_1->ring2->setPatchName("index2b");


    // ring3
    R_t_2_f.RotationZ(PI / 4.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_1->ring3->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF1Dof3");
    tekpatch_set_1->ring3->setPatchName("index3");
//    tekpatch_set_1->ring3->setThreshold(15.0); // change it to a different one  ?

    // --------------------------------------------------------------
    // The normal is now right, but not the horizontal position
    // Finger 1 OF ALLEGRO HAND
    // / MEDIUM OF ALLEGRO HAND
    // thumb1
    //    R_t_2_f.RotationY(PI);
    R_t_2_f.RotationYXZ(PI, PI, PI);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);

    tekpatch_set_2->thumb1->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF2Dof1");
    tekpatch_set_2->thumb1->setPatchName("medium1a");

    // thumb2
    //    R_t_2_f.RotationY(PI);
    R_t_2_f.RotationYXZ(PI, PI, PI);
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.043, d_half_finger_width);

    tekpatch_set_2->thumb2->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF2Dof1");
    tekpatch_set_2->thumb2->setPatchName("medium1b");

    // -------------------------------------------------------

    // medium1
    R_t_2_f.Identity();
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.017, d_half_finger_width);

    tekpatch_set_2->medium1->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF2Dof2");
    tekpatch_set_2->medium1->setPatchName("medium2a");


    // medium3
    R_t_2_f.RotationZ(-PI / 2.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.025, d_half_finger_width);

    //  tempVec3.Set(0.0, 0.017, 0.003);
    tekpatch_set_2->medium2->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF2Dof2");
    tekpatch_set_2->medium2->setPatchName("medium2b");


    // medium2
    R_t_2_f.RotationZ(PI / 4.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_2->medium3->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF2Dof3");
    tekpatch_set_2->medium3->setPatchName("medium3");


    // / Finger 3 OF ALLEGRO HAND
    // index0
    R_t_2_f.Identity();
    R_g_2_f = R_gaz1_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_2->index0->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF3Dof1");

    //  tekpatch_set_1->index0->setPatchInfo(R_g_2_f, R_gaz1_2_t * tempVec3, "HandF2Dof1");
    tekpatch_set_2->index0->setPatchName("pinky1b");


    // index1
    R_t_2_f.Identity();
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.017, d_half_finger_width);
    tekpatch_set_2->index1->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF3Dof2");
    tekpatch_set_2->index1->setPatchName("pinky2a");


    // index3
    R_t_2_f.RotationZ(PI / 2.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.025, d_half_finger_width);
    tekpatch_set_2->index2->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF3Dof2");
    tekpatch_set_2->index2->setPatchName("pinky2b");


    // index2
    R_t_2_f.RotationZ(-PI / 4.0);
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.03, d_half_finger_width);
    tekpatch_set_2->index3->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF3Dof3");
    tekpatch_set_2->index3->setPatchName("pinky3");


    // / Add patches on that patch set ...
    //  tekpatch_set_2->thumb0->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF3Dof3");
    //  tekpatch_set_2->thumb0->setPatchName("newThumb");


    // TODO: set right the patches

    // thumb before tip
    R_t_2_f.Identity();
    R_g_2_f = R_gaz23_2_t * R_t_2_f;

    //  tempVec3.Set(0.0, 0.02, d_half_finger_width);
    tempVec3.Set(0.0, 0.035, d_half_finger_width);
    tekpatch_set_1->thumb1->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF4Dof3");
    tekpatch_set_1->thumb1->setPatchName("thumb_3a");

    // thumb tip
    R_t_2_f.Identity();
    R_g_2_f = R_gaz23_2_t * R_t_2_f;
    tempVec3.Set(0.0, 0.045, d_half_finger_width);
    tekpatch_set_1->thumb2->setPatchInfo(R_g_2_f, R_gaz23_2_t * tempVec3, "HandF4Dof3");
    tekpatch_set_1->thumb2->setPatchName("thumb_3b");


    // thumb base (needs to be checked ..)
    R_g_2_f.Identity();
    tempVec3.Set(d_half_finger_width, -0.015, 0.0);
    R_g_2_f.Set(0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0);

    tekpatch_set_1->thumb0->setPatchInfo(R_g_2_f, tempVec3, "HandF4Dof2");
    tekpatch_set_1->thumb0->setPatchName("thumb_1");


    // / ---------------------


    // / Create the full vector to access all of them ...
    full_patch_list.clear();
    full_patch_list.insert( full_patch_list.end(), tekpatch_set_1->mPatchList.begin(), tekpatch_set_1->mPatchList.end() );

    // Only add this one in simulation. On the real robot, not using the thumb yet
    // This is bs. Using the thumb should not depend on running in simu or real hand.
    // add anyway
    //    if (!b_use_yarp)
    full_patch_list.insert( full_patch_list.end(), tekpatch_set_2->mPatchList.begin(), tekpatch_set_2->mPatchList.end() );

    for (int i = 0; i < full_patch_list.size(); i++) {
        full_patch_list[i]->setThreshold(d_temp_4);
    }


    // contact dof on finger:
    v_finger_contact_max_dof.Resize(NB_FINGERS); /* Nb of fingers*/
    v_finger_contact_max_dof.One();
    v_finger_contact_max_dof.SMinus();

    //  v_finger_contact_max_dof_id.Resize(NB_FINGERS);
    //  v_finger_contact_max_dof_id.Zero();


    /** =================================
     * =  INIT ALLEGRO HAND and KUKA: Variables and ALLEGROHAND START   =
     * ================================= */

    /* Allegro Hand Specific*/
    v_curr_torque_a.Resize(NB_DOF_HAND, false);
    v_curr_torque_a.Zero();

    v_total_torque_a.Resize(NB_DOF_HAND, false);
    v_total_torque_a.Zero();


    v_est_joint_torques.resize(NB_DOF_ARM);
    v_est_endeff_torques.resize(6);

    v_position_a.Resize(NB_DOF_HAND, false);
    v_position_a.Zero();
    v_position_a_old.Resize(NB_DOF_HAND, false);
    v_position_a_old.Zero();

    //  for(int i=0; i < NB_DOF_HAND; i ++){
    allegro_hand_state_desired_torque.effort.resize(NB_DOF_HAND);
    allegro_hand_state_desired_torque.velocity.resize(NB_DOF_HAND);
    allegro_hand_state_desired_torque.position.resize(NB_DOF_HAND);
    allegro_hand_state_desired_torque.name.resize(NB_DOF_HAND);


    std::string jointNames[NB_DOF_HAND] =
    {
        "joint_0.0",  "joint_1.0",  "joint_2.0",   "joint_3.0",
        "joint_4.0",  "joint_5.0",  "joint_6.0",   "joint_7.0",
        "joint_8.0",  "joint_9.0",  "joint_10.0",  "joint_11.0",
        "joint_12.0", "joint_13.0", "joint_14.0",  "joint_15.0"
    };


    allegro_hand_state_info.effort.resize(NB_DOF_HAND);
    allegro_hand_state_info.velocity.resize(NB_DOF_HAND);
    allegro_hand_state_info.position.resize(NB_DOF_HAND);
    allegro_hand_state_info.name.resize(NB_DOF_HAND);


    for (int i = 0; i < NB_DOF_HAND; i++) {
        allegro_hand_state_desired_torque.name[i] = jointNames[i];
        allegro_hand_state_info.name[i]           = jointNames[i];
    }


    robot_state_info.effort.resize(NB_DOF_TOT);
    robot_state_info.velocity.resize(NB_DOF_TOT);
    robot_state_info.position.resize(NB_DOF_TOT);
    robot_state_info.name.resize(NB_DOF_TOT);

    for (int i = 0; i < NB_DOF_TOT; i++) {
        robot_state_info.name[i] = stdv_jointnames[i];
    }


    if ( this->rosnode_->hasParam("my_gains_pd") ) {
        ROS_INFO("CTRL: PD gains loaded from param server.");

        for (int i = 0; i < NB_DOF_HAND; i++) {
            this->rosnode_->getParam(pGainParams[i], k_p[i]);
            this->rosnode_->getParam(dGainParams[i], k_d[i]);
        }
    }
    else {
        // gains will be loaded every control iteration
        ROS_FATAL("CTRL: PD gains not loaded");
        ROS_WARN("Check launch file is loading /parameters/my_gains_pd.yaml");
        ROS_WARN("Loading default PD gains...");
        exit(0);
    }


    /* Shared Kuka and Hand */
    v_gravComp_torque.Resize(NB_DOF_TOT, false);
    v_gravComp_torque.Zero();
    v_additionnal_torques.Resize(NB_DOF_TOT, false);
    v_additionnal_torques.Zero();
    v_additionnal_torque_friction.Resize(NB_DOF_TOT, false);
    v_additionnal_torque_friction.Zero();
    v_contact_torque.Resize(NB_DOF_TOT, false);
    v_contact_torque.Zero();
    v_virtual_contact_torques.Resize(NB_DOF_TOT, false);
    v_virtual_contact_torques.Zero();

    v_total_torque.Resize(NB_DOF_TOT, false);
    v_total_torque.Zero();

    v_curr_position.Resize(NB_DOF_TOT, false);
    v_curr_position.Zero();

    v_curr_torques.Resize(NB_DOF_TOT, false);


    v_ravin_joint_pos_tracked.resize(NB_DOF_TOT);
    v_ravin_joint_pos_tracked.setZero();

    v_explore_joint_pos_simple.resize(NB_DOF_TOT);
    v_explore_joint_pos_simple.setZero();


    v_ravin_joint_pos_grasped.resize(NB_DOF_TOT);
    v_ravin_joint_pos_grasped.setZero();

    v_curr_position_tempgaz.Resize(NB_DOF_TOT);
    v_initial_position.Resize(NB_DOF_TOT, false);
    v_initial_position.Zero();
    v_prev_position.Resize(NB_DOF_TOT, false);
    v_prev_position.Zero();
    v_curr_position_actuator.Resize(NB_DOF_TOT, false);
    v_curr_position_actuator.Zero();

    v_last_position_static.Resize(NB_DOF_TOT, false);
    v_last_position_static.Zero();
    v_curr_velocity.Resize(NB_DOF_TOT, false);
    v_curr_velocity.Zero();

    v_current_velocity_filtered_2deg.resize(NB_DOF_TOT);
    v_current_position_filtered_2deg.resize(NB_DOF_TOT);
    v_curr_velocity_filtered_CDD.Resize(NB_DOF_TOT);
    v_prev_velocity_filtered_CDD.Resize(NB_DOF_TOT);
    v_curr_velocity_filtered_CDD2.Resize(NB_DOF_TOT);
    v_curr_acceleration_filtered_CDD2.Resize(NB_DOF_TOT);
    v_curr_acceleration_filtered_CDD1.Resize(NB_DOF_TOT);
    v_prev_velocity.Resize(NB_DOF_TOT, false);
    v_prev_velocity.Zero();
    v_desired_velocity.Resize(NB_DOF_TOT, false);
    v_desired_velocity.Zero();

    v_torques_coriolis.Resize(NB_DOF_TOT);
    v_torques_coriolis.Zero();

    v_cart_pos_tracked.setZero();
    v_cart_pos_tracked_filtered.setZero();

    // /////////////////////////////////

    /*
     * canDevice                   = new controlAllegroHand();
     * b_allegro_success           = false;
     * b_allegro_connected_once    = false;
     * b_allegro_data_micro_change = false;
     *
     * //  bool b_use_allegro = false;
     * b_use_allegro = false;
     * getParamSafeHandle("b_use_allegro", b_use_allegro);
     *
     * if (b_use_allegro) {
     * canDevice->init();
     * cout << "Finished allegro hand init" << endl;
     *
     * v_total_torque.Zero();
     * v_total_torque_a = v_total_torque.GetSubVector(NB_DOF_ARM, NB_DOF_HAND);
     * cout << "Trying to set torques" << endl;
     * canDevice->setTorque( v_total_torque_a.Array() );
     * cout << "Trying to write devices torques" << endl;
     *
     * canDevice->_writeDevices();
     * cout << "Finished writedevices" << endl;
     * b_allegro_connected_once = true;
     * b_allegro_success        = true;
     * }
     */


    /** ------------------
     * INIT Inv Dyn (for allegrohand Gravity compensation)
     * ---------------------- */
    mInvDynamics.SetRobot(mRobot);
    mInvDynamics.Init();
    mInvDynamics.SetGravityCompensationOnly(true);


    /** ---------------------------------------
     * INIT sensors, joints, "actuators"
     * ----------------------------------------- */
    mSensorsGroup.SetSensorsList( mRobot->GetSensors() );
    mActuatorsGroup.SetActuatorsList( mRobot->GetActuators() );

    // / Some Debug
    //    cout << "sensor count: " <<mRobot->GetSensorsCount() << endl;
    //    cout << "actuators count: " <<mRobot->GetActuatorsCount() << endl;
    //    cout << "dof count: " <<mRobot->GetDOFCount() << endl;
    //    cout << "links count: " <<mRobot->GetLinksCount() << endl;
    //    cout << "type: " <<mRobot->GetType() << endl;
    //    cout << "subtype: " <<mRobot->GetSubType() << endl;

    /** =================================
     * =     INIT LINKS AND STUFF      =
     * ================================= */

    // / Display link names
    if (false)
        for (int i = 0; i < mRobot->GetLinksCount(); i++) {
            cout << "-----------------" << endl;
            cout << "link " << i << ": " << mRobot->GetLinks()[i]->GetName() << endl;
            cout << "Joint " << i << ", axis num: " << ( (RevoluteJoint *)(mRobot->GetJoints()[i]) )->mAxis << endl;
            cout << "axis:";
            ( (RevoluteJoint *)(mRobot->GetJoints()[i]) )->GetAxis().Print();
            double min, max;
            ( (RevoluteJoint *)(mRobot->GetJoints()[i]) )->GetJointLimits(min, max);
        }

    nBaseID = 0;


    /** ==========================
     * =          OTHER         =
     * ========================== */

    /** -------------------
    *    Control modes
    * ------------------ */

    jointCtrlMode     = new CtrlMode[NB_DOF_TOT];
    prevJointCtrlMode = new CtrlMode[NB_DOF_TOT];

    for (int i = 0; i < NB_DOF_TOT; i++) {
        jointCtrlMode[i]     = TORQUE;
        prevJointCtrlMode[i] = TORQUE;
    }

    ctrlm_robot = GRAV_COMP; // Start the Kuka in Gravity Compensation mode
    //  ctrlm_robot_next=POSITION_PID_STATIC;
    ctrlm_robot_next = GRAV_COMP;
    ctrlm_robot_prev = GRAV_COMP;

    b_fingers_should_open     = new bool[4];
    b_fingers_should_open_old = new bool[4];
    for (int i =0;i<NB_FINGERS;i++){
        b_fingers_should_open[i]= false;
        b_fingers_should_open_old[i]= false;
    }

    b_auto_pos_tracking           = false;
    b_use_virtual_contact_torques = true;
    getParamSafeHandle("b_use_virtual_contact_torques", b_use_virtual_contact_torques);
    b_auto_next_point = false;

    //  b_cart_command_ready = false;

    // Deactivate these for now
    b_use_joint_centering = false;
    getParamSafeHandle("b_use_joint_centering", b_use_joint_centering);
    getParamSafeHandle("b_avoid_joint_limits",  b_avoid_joint_limits);


    //    b_avoid_joint_limits     = false;
    b_gazebo_object_imp_zero = false;

    b_use_point_tracking = false;
    getParamSafeHandle("b_spacem_trackpoint", b_use_point_tracking);

    getParamSafeHandle("b_use_vc_normalinfo", b_use_vc_normalinfo);

    getParamSafeHandle("d_use_vc_normalinfo", d_use_vc_normalinfo);


    d_joint_position_kp = 0.005;
    d_joint_position_kd = 0.01;
    getParamSafeHandle("d_joint_position_kp", d_joint_position_kp);
    getParamSafeHandle("d_joint_position_kd", d_joint_position_kd);


    // Get contact levels from YAML file

    XmlRpc::XmlRpcValue xmlrpc_contact_levels;
    std::string s_contact_levels = "contact_levels";
    this->rosnode_->getParam(s_contact_levels, xmlrpc_contact_levels);

    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    contact_levels.clear();

    for (i = xmlrpc_contact_levels.begin(); i != xmlrpc_contact_levels.end(); ++i) {
        std::vector<int> thislist;
        this->rosnode_->getParam(s_contact_levels + "/" + i->first, thislist);
        contact_levels.push_back(thislist);
    }

    // Display contact levels
    for (auto& i : contact_levels) {
        cout << "new level:" << endl;

        for (auto& j : i) {
            cout << "j:" << j << endl;
        }
    }



    // ------------------------------------
    XmlRpc::XmlRpcValue xmlrpc_keypoints;
    std::string s_keypoints = "keypoints";
    this->rosnode_->getParam(s_keypoints, xmlrpc_keypoints);


    // Read desired contact points from yaml parameter server
    stdvec_des_cart_positions.clear();

    for(int i=0; i<xmlrpc_keypoints.size();i++){
        Eigen::Vector3d new_point;
        for(int j=0; j<xmlrpc_keypoints[i].size();j++){
            new_point(j)=xmlrpc_keypoints[i][j];
            cerr << "i,j:" << i << "," << j <<": " << xmlrpc_keypoints[i][j] << endl;
        }

        stdvec_des_cart_positions.push_back(new_point);
    }
    i_curr_despos_i = 0;


    for(auto point:stdvec_des_cart_positions){
        cerr << "new point: " << point << endl;
    }

    //    cerr << "reached here " << endl;

    //    exit(0);

    /** -------------------
     * Inits for PID
     * ---------------------- */


    v_pos_KP.Resize(NB_DOF_TOT, false);
    v_pos_KD.Resize(NB_DOF_TOT, false);
    v_pos_KI.Resize(NB_DOF_TOT, false);

    v_joint_error.Resize(NB_DOF_TOT, false);
    v_joint_error.Zero();
    v_joint_error_prev.Resize(NB_DOF_TOT, false);
    v_joint_error_deriv.Resize(NB_DOF_TOT, false);
    v_joint_error_integral.Resize(NB_DOF_TOT, false);
    v_joint_error_integral.Zero();
    v_torques_for_joint_error.Resize(NB_DOF_TOT, false);

    //  v_desired_pos_rad.Resize(NB_DOF_TOT, false);
    v_desired_pos_rad_Filtered.Resize(NB_DOF_TOT, false);
    v_prev_desired_pos_rad_Filtered.Resize(NB_DOF_TOT, false);
    v_des_diff_rad_Filtered.Resize(NB_DOF_TOT, false);

    v_desired_hand_pos_filtered.Resize(NB_DOF_HAND, false);
    v_desired_arm_pos_filtered.Resize(NB_DOF_ARM, false);

    v_des_diff_rad_Filtered.Zero();

    // new values because unstable ...
    d_pos_KP = 0.01;
    d_pos_KD = 0.005;


    for (int i = 0; i < NB_DOF_ARM; i++)
        for (int j = 0; j < 4; j++) {
            stdvec_indices_finger[j].push_back(i);
        }

    for (int i = 0; i < NB_DOF_FINGER; i++)
        for (int j = 0; j < 4; j++) {
            stdvec_indices_finger[j].push_back(NB_DOF_ARM + NB_DOF_FINGER * j + i);
        }


    /** -------------------
     * Other Inits
     * ---------------------- */

    dt = 0.001; // how to obtain this otherwise ?? // seems to be problematic above 2ms...

    b_init      = 1;
    i_counter   = 0;
    i_no_torque = 10; // no torque sent the first second ....

    fingerReachedLimit.Resize(4, false);

    b_reset_offset = false;
    i_nb_contacts  = 0;

    v3_dir_p0_to_p1.Zero();
    v3_dir_p0_to_p1[0] = 1.0;

    b_only_gravity_compensation = 0; // at least for the simulation, it's better to start with this mode

    // TODO YAML
    //  d_desired_normal_pressure=0.5; // If too high, pressure too high, if too low, not following ...
    d_desired_normal_pressure = 1.0;          // If too high, pressure too high, if too low, not following ...
    getParamSafeHandle("d_desired_normal_pressure",         d_desired_normal_pressure);

    //    cout << "d_desired_normal_pressure after getparam: " << d_desired_normal_pressure << endl;
    //    exit(0);
    d_desired_virtual_normal_velocity = 30.0; // high enough (1.0 was not enough, at least in simulation, now 1 is good)
    getParamSafeHandle("d_desired_virtual_normal_velocity", d_desired_virtual_normal_velocity);
    b_arm_for_virtual_force = true;           // lets start this way

    b_was_disabled = 1;

    mJointMotionAmplitude.Resize(NB_DOF_TOT, false);
    mJointMotionMean.Resize(NB_DOF_TOT, false);

    //  mTimePeriod.Resize(NB_DOF_TOT, false);
    mPhaseShift.Resize(NB_DOF_TOT, false);
    mPeriodicMinAngle.Resize(NB_DOF_TOT, false);
    mPeriodicMaxAngle.Resize(NB_DOF_TOT, false);
    mDesiredPositionDeg.Resize(NB_DOF_TOT, false);

    //  bStoppingProcess = false;

    mRestPosition.Resize(NB_DOF_TOT, false);
    mRestPosition.Zero();

    startTime = GetClock().GetTime();

    d_lasttime_button0 = startTime;
    d_lasttime_button1 = startTime;

    getParamSafeHandle("b_use_gazebo",                 b_use_gazebo);
    getParamSafeHandle("b_control_real_robot",         b_control_real_robot);

    getParamSafeHandle("b_use_predetermined_pd_gains", b_use_predetermined_pd_gains);
    cout << "b_use_gazebo before : " << b_use_gazebo << endl;

    i_a_hand_counter = 0;


    // Load hand parameters
    std::string s_hand_pos_init[NB_DOF_HAND] =
    {
        "hand_pos/init/j00", "hand_pos/init/j01", "hand_pos/init/j02", "hand_pos/init/j03",
        "hand_pos/init/j10", "hand_pos/init/j11", "hand_pos/init/j12", "hand_pos/init/j13",
        "hand_pos/init/j20", "hand_pos/init/j21", "hand_pos/init/j22", "hand_pos/init/j23",
        "hand_pos/init/j30", "hand_pos/init/j31", "hand_pos/init/j32", "hand_pos/init/j33"
    };

    std::string s_hand_pos_final[NB_DOF_HAND] =
    {
        "hand_pos/final/j00", "hand_pos/final/j01", "hand_pos/final/j02", "hand_pos/final/j03",
        "hand_pos/final/j10", "hand_pos/final/j11", "hand_pos/final/j12", "hand_pos/final/j13",
        "hand_pos/final/j20", "hand_pos/final/j21", "hand_pos/final/j22", "hand_pos/final/j23",
        "hand_pos/final/j30", "hand_pos/final/j31", "hand_pos/final/j32", "hand_pos/final/j33"
    };

    v_hand_open_posture.resize(NB_DOF_HAND);
    v_hand_close_posture.resize(NB_DOF_HAND);
    v_hand_explo_posture.resize(NB_DOF_HAND);
    v_hand_explo_posture.setZero();
    b_got_hand_explo_posture = false;

    for (int i = 0; i < NB_DOF_HAND; i++) {
        getParamSafeHandle(s_hand_pos_init[i],  v_hand_open_posture(i),  false);
        getParamSafeHandle(s_hand_pos_final[i], v_hand_close_posture(i), false);
    }


    m_jsim_tot.Resize(NB_DOF_TOT, NB_DOF_TOT);
    m_jsim_tot.Identity();
    mat_JSIM_f0.Resize(NB_DOF_ARM + NB_DOF_FINGER, NB_DOF_ARM + NB_DOF_FINGER);
    mat_JSIM_f1.Resize(NB_DOF_ARM + NB_DOF_FINGER, NB_DOF_ARM + NB_DOF_FINGER);
    mat_JSIM_f2.Resize(NB_DOF_ARM + NB_DOF_FINGER, NB_DOF_ARM + NB_DOF_FINGER);
    mat_JSIM_f3.Resize(NB_DOF_ARM + NB_DOF_FINGER, NB_DOF_ARM + NB_DOF_FINGER);
    mat_JSIM_f0.Identity();
    mat_JSIM_f1.Identity();
    mat_JSIM_f2.Identity();
    mat_JSIM_f3.Identity();


    jointspace_inertia_f0.resize(NB_DOF_ARM + NB_DOF_FINGER);
    jointspace_inertia_f1.resize(NB_DOF_ARM + NB_DOF_FINGER);
    jointspace_inertia_f2.resize(NB_DOF_ARM + NB_DOF_FINGER);
    jointspace_inertia_f3.resize(NB_DOF_ARM + NB_DOF_FINGER);

    kdl_jntarray_f0.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_f1.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_f2.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_f3.resize(NB_DOF_ARM + NB_DOF_FINGER);


    kdl_jntarray_vel_f0.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_vel_f1.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_vel_f2.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_vel_f3.resize(NB_DOF_ARM + NB_DOF_FINGER);


    kdl_jntarray_coriolis_f0.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_coriolis_f1.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_coriolis_f2.resize(NB_DOF_ARM + NB_DOF_FINGER);
    kdl_jntarray_coriolis_f3.resize(NB_DOF_ARM + NB_DOF_FINGER);


    v_joint_contact_info.Resize(NB_DOF_HAND, 6, false); /* number of information: 4 for now*/

    b_compensate_coriolis = false;                      // THIS IS BUGGY , DO NOT USE

    b_temp_switch = false;


    finger_enabled.Resize(NB_FINGERS);
    finger_enabled.Zero();
    finger_enabled[0] = ENABLE_FINGER_0;
    finger_enabled[1] = ENABLE_FINGER_1;
    finger_enabled[2] = ENABLE_FINGER_2;
    finger_enabled[3] = ENABLE_FINGER_3;


    v_cart_endeff_orient.resize(3, 3);
    v_cart_endeff_pos.resize(3);


    /** -----------------------------------------
    * Define Vectors with zeros on the arm
    * ---------------------------------------- */
    v_zeros_on_arm.Resize(NB_DOF_TOT, false);

    for (int i = 0; i < NB_DOF_TOT; i++) {
        if (i < NB_DOF_ARM) {
            v_zeros_on_arm[i] = 0.0;
        } else {
            v_zeros_on_arm[i] = 1.0;
        }
    }

    cout << "## Getting values from the parameter server" << endl;
    getParamSafeHandle("b_debug",              b_debug);
    getParamSafeHandle("i_contact_level_init", i_contact_level);
    cout << "i_contact_level: " << i_contact_level << endl;


    v_contact_on_link_allowed.Resize(NB_DOF_TOT);

    setContactLevel2(i_contact_level, contact_levels);


    v_cart_desired_pos_final.resize(3);
    v_cart_desired_pos_final.setZero();
    v_cart_desired_pos_tracked.resize(3);
    v_cart_desired_pos_tracked.setZero();

    b_reaching_trajectory = false;

    b_auto_orientation = true;
    getParamSafeHandle("b_auto_orientation", b_auto_orientation);


    //    d_joint_damping_coefficient = 150.0; // TO BE TESTED !!!

    getParamSafeHandle("b_use_isotropic_impedance",   b_use_isotropic_impedance);
    getParamSafeHandle("d_impedance_lin_stiff",       d_impedance_lin_stiff);
    getParamSafeHandle("d_impedance_lin_damp",        d_impedance_lin_damp);
    getParamSafeHandle("d_impedance_rot_stiff",       d_impedance_rot_stiff);
    getParamSafeHandle("d_impedance_rot_damp",        d_impedance_rot_damp);
    getParamSafeHandle("b_simulate_tekscan",          b_simulate_tekscan);
    getParamSafeHandle("d_joint_damping_coefficient", d_joint_damping_coefficient);
    getParamSafeHandle("b_ravin_task_mode_allowed",   b_ravin_task_mode_allowed);

    b_task_mode_signal = false;
    b_close_signal     = false;
    b_close_allowed    = false;
    b_tighten_signal   = false;
    b_task_rotation    = false;
    b_task_hammering   = false;

    b_ravin_cart_reach = false;


    b_use_ravins_method_for_vc = false;
    getParamSafeHandle("b_use_ravins_method_for_vc", b_use_ravins_method_for_vc, true);

    d_ravin_finger_orient_stiffness = 0.001;

    getParamSafeHandle("d_ravin_finger_o_stiffness", d_ravin_finger_orient_stiffness);

    cerr << "deb5"  << endl;

    myImpedanceController = new ImpedanceController();
    myImpedanceController->InitChain(kdlchain_endeff_7_link);

    if ( GetConsole() ) {
        AddConsoleCommand("ro");           // reset the tekscan offset
        AddConsoleCommand("gr");           // only grav-comp
        AddConsoleCommand("gt");           // enable other torques
        AddConsoleCommand("setp");         // set normal pressure value for a contact
        AddConsoleCommand("setthreshold"); // set normal pressure value for a contact
        AddConsoleCommand("setvforce");    // set normal pressure value for a contact
        AddConsoleCommand("setwn");        // set normal pressure value for a contact
        AddConsoleCommand("setcounter");   // set
        AddConsoleCommand("s");            // shut down ...

        // Saving
        AddConsoleCommand("dnam");         // set name for saving
        AddConsoleCommand("p");            // set next saving number
        AddConsoleCommand("save");         // start/stop saving
        AddConsoleCommand("'");            // start/stop saving (2nd one ...)

        AddConsoleCommand("bpt");          // use projected torques
        AddConsoleCommand("bt");           // b_auto_pos_tracking: follow point until next desired point ...
        AddConsoleCommand("bs");           // b_auto_orientation


        AddConsoleCommand("kg");           // set in Grav Comp (KUKA)
        AddConsoleCommand("ks");           // set in Static PID (KUKA)
        AddConsoleCommand("kt");           // set in Torque (KUKA)
        AddConsoleCommand("setkp");        //
        AddConsoleCommand("setkd");        //
        //    AddConsoleCommand("kp"); // For Cartesian
        //    AddConsoleCommand("kd"); //
        //    AddConsoleCommand("ki"); //
        //    AddConsoleCommand("kpj"); // For Joint
        //    AddConsoleCommand("kdj"); //
        //    AddConsoleCommand("kij"); //
        AddConsoleCommand("ba");  // Toggle arm in virtual torques

        AddConsoleCommand("vc");  // Toggle virtual torques
        //    AddConsoleCommand("bc"); // Toggle Coriolis torques
        AddConsoleCommand("bc");  // Toggle other value for debug

        AddConsoleCommand("l");   // set the level of contacts desired
        //    AddConsoleCommand("sc"); // start a scan in some direction
        AddConsoleCommand("k");   // Change gains for the impedance controller
        AddConsoleCommand("des"); // Change gain for joint pid controller (whole arm ...)
        AddConsoleCommand("g");   // Change gain for joint pid controller (whole arm ...)
        AddConsoleCommand("n");   // Change gain for joint pid controller (whole arm ...)

        AddConsoleCommand("bcj"); // Add task to center joints
        AddConsoleCommand("bjl"); // Add task to avoid joint limits
        AddConsoleCommand("gi");  // Add task to avoid joint limits
        AddConsoleCommand("gd");  // Get direction ...
        AddConsoleCommand("go");  // Get direction ...
        AddConsoleCommand("gb");  // Get direction ...
        AddConsoleCommand("bpin");
    }

    cerr << "deb5a"  << endl;

    /** ==========================
     * =  ROBOT UPDATE for INIT =
     * ========================== */

    // Read starting robot position
    mSensorsGroup.ReadSensors();
    v_curr_position    = mSensorsGroup.GetJointAngles();
    v_curr_torques     = mSensorsGroup.GetJointTorques();
    v_initial_position = v_curr_position;


    if ( (mRobot->GetControlMode() != Robot::CTRLMODE_JOINTIMPEDANCE) && b_control_real_robot ) {
        ctrlm_robot_next = POSITION_PID_STATIC;
        ctrlm_robot_prev = GRAV_COMP;
        ROS_WARN_STREAM("Switching to CTRLMODE_JOINTIMPEDANCE AT THE BEGGINNING ");
    }

    mRobot->UpdateLinks();

    cerr << "deb5b"  << endl;

    /** ==========================
     * =     FILTERS SETUP      =
     * ========================== */

    // Setup Filter for oscillating finger motion
    wn = 1.0; // frequency ...

    getParamSafeHandle("wn_joint_pos_arm_filter",  wn);
    cdd_desired_joint_pos_arm = new CDDynamics(NB_DOF_ARM, dt, wn);

    getParamSafeHandle("wn_joint_pos_hand_filter", wn);
    cdd_desired_joint_pos_hand = new CDDynamics(NB_DOF_HAND, dt, wn);


    // Setup position filter
    double wn_filter;
    wn_filter = 30.0; // this is pretty sensitive ...
    getParamSafeHandle("wn_position_filter", wn_filter);
    cdd_positionFilter = new CDDynamics(NB_DOF_TOT, dt, wn_filter);
    cdd_positionFilter->SetStateTarget(v_initial_position, v_initial_position);


    // Setup Velocitiy filter
    wn_filter = 30.0;
    getParamSafeHandle("wn_velocity_filter", wn_filter);
    cdd_velocityFilter = new CDDynamics(NB_DOF_TOT, dt, wn_filter);
    cdd_velocityFilter->SetStateTarget(v_curr_velocity, v_curr_velocity);

    // Setup Velocity filter
    wn_filter = 5.0;
    getParamSafeHandle("wn_contact_normal_filter", wn_filter);
    cdd_contact_normal_filter = new CDDynamics(3, dt, wn_filter);
    cdd_contact_normal_filter->SetStateTarget( MathLib::Vector3(1.0, 0.0, 0.0), MathLib::Vector3(1.0, 0.0, 0.0) );

    wn_filter = 5.0;
    getParamSafeHandle("wn_max_intensity_filter", wn_filter);
    v_max_intensity.Resize(1);
    v_max_intensity_filtered.Resize(1);
    v_max_intensity(0) = 1.0;
    cdd_max_intensity  = new CDDynamics(1, dt, wn_filter);
    cdd_max_intensity->SetStateTarget(v_max_intensity, v_max_intensity);


    // Setup Velocity filter
    wn_filter = 1.0;
    getParamSafeHandle("wn_cartesian_position_motion", wn_filter);
    cdd_cartesian_position_motion = new CDDynamics(3, dt, wn_filter);
    cdd_cartesian_position_motion->SetStateTarget( MathLib::Vector3(1.0, 0.0, 0.0), MathLib::Vector3(1.0, 0.0, 0.0) );

    wn_filter = 1.0;
    getParamSafeHandle("wn_cartesian_orientation_motion", wn_filter);
    cdd_cartesian_orientation_motion = new CDDynamics(3, dt, wn_filter);
    cdd_cartesian_orientation_motion->SetStateTarget( MathLib::Vector3(1.0, 0.0, 0.0), MathLib::Vector3(1.0, 0.0, 0.0) );

    cdd_cartesian_orientationQ_motion = new CDDynamics(4, dt, wn_filter);
    cdd_cartesian_orientationQ_motion->SetStateTarget( MathLib::Vector(4).One(), MathLib::Vector(4).One() );

    cerr << "deb5c"  << endl;

    /** ===========================================
     * =  INIT CONTACT POINTS, COMPUTE JACOBIAN  =
     * =========================================== */

    unsigned int number_of_contacts_max=25.0;
    list_contacts.resize(number_of_contacts_max);

    // pre initilize the tactileContact pointers' vector
    for (int i = 0; i < number_of_contacts_max; i++) {
        list_contacts[i] = new TactileContact();
    }

    list_links.resize(NB_DOF_TOT);


    // Initialize desired pressure
    for (int i = 0; i < NB_DOF_TOT; i++) {
        list_links[i]                                 = new TactileContact();
        list_links[i]->d_desired_pressure             = d_desired_normal_pressure;
        list_links[i]->d_desired_virtual_vel_velocity = d_desired_virtual_normal_velocity;
        list_links[i]->s_link_name_urdf               = stdvec_link_names_gazebo[i]; // / new
    }


    // it's reactivated when I get a message from Ravin.
    b_ravin_jointpos_reach = false;

    // activated when the hand should start closing
    b_ravin_close_mode = false;

    // activated when the robot should be doing its task
    b_ravin_task_mode = false;

    // activated when an object is grasped (compliance finished)
    b_ravin_grasped_mode = false;

    // Activate the closing of the fingers by default (when reached position)
    b_ravins_mode = true;


    // / from here, this is not used anymore
    mClock.Reset();
    mClock.SetInternal(true);

    // let the prog run
    b_paused = false;
    cerr << "deb5d"  << endl;

    return STATUS_OK;
}

RobotInterface::Status RobotHapticControllerClass::RobotFree() {
    cout << "-Robot Free-" << endl;

    queue_.clear();
    queue_.disable();

    cout << "-Robot Free end -" << endl;

    return STATUS_OK;
}

RobotInterface::Status RobotHapticControllerClass::RobotStart() {
    return STATUS_OK;
}

RobotInterface::Status RobotHapticControllerClass::RobotStop() {
    cout << "-Robot Stop-" << endl;


    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < NB_DOF_TOT; i++) {
        array.data.push_back(0.0); // pb: This contains gravity compensation torques for _0hand ...
    }

    pub_torques.publish(array);


    cout << "Closing robot" << endl;
    v_total_torque.Zero();

    return STATUS_OK;
}

/**
 *
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *
 #####  #  ##  ####          ####   #  ##  ####  ######
 ##     ## ##  ## ##          ##    ## ##   ##     ##
 ##     #####  ## ##          ##    #####   ##     ##
 ####   #####  ## ##          ##    #####   ##     ##
 ##     #####  ## ##          ##    #####   ##     ##
 ##     ## ##  ## ##          ##    ## ##   ##     ##
 #####  ##  #  ####          ####   ##  #  ####    ##
 #####
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *  ==================================================================================================================================
 *
 * */
RobotInterface::Status RobotHapticControllerClass::RobotUpdate() {




    ros::spinOnce(); // Run the Callback functions

    if (!b_paused) RobotUpdateRun();
}

RobotInterface::Status RobotHapticControllerClass::RobotUpdateRun() {
    /**  ==========================
    * =     String display     =
    * ========================== */

    double d_time_since_last_loop = (mClock.GetTime() - d_lasttime_loopend);

    {
        //    boost::lock_guard<boost::mutex> guard(bm_mutex_main); // lock this: --> do not access the full jacobian ... // sure for guard instead of lock ??
        std::lock_guard<std::mutex> guard_c(bm_mutex_main_c); // lock this: --> do not access the full jacobian ... // sure for guard instead of lock ??

        // / Time tracking display
        if (b_display_time) cout << ossT.str() << flush;      // Write and flush the string

        // / Data debug display
        ossG << "------------------------>>>" << endl;
        ossG << std::setprecision(3);
        ossG << std::fixed;

        cout << ossGv.str();         // add that line or not at the end ...
        cout << ossG.str() << flush; // Write and flush the string
        ossG.str("");                // Empty the buffer stream
        ossGv.str("");
        ossDebug.str("\n");
        ossDebug << endl;
    }

    /**  =================================
    * =              CLOCKS           =
    * ================================= */

    // / Update Clock, this code is usefull to track slow parts of the program and general speed information
    mClock.Update();
    lastTime = mClock.GetTime();

    double d_time_for_last_loop = (mClock.GetTime() - d_lasttime_loopstart);
    ossG << "\n\n<<<-------------------------\n ==> Time since last RobotUpdate:\t" << d_time_for_last_loop * 1000 << "ms <==" << endl;
    ossT << "\n\n<<<-------------------------\n ==> Time since last RobotUpdate:\t" << d_time_for_last_loop * 1000 << "ms <==" << endl;

    ossT << "time since end of last loop: " << (mClock.GetTime() - d_lasttime_loopend) * 1000.0 << endl;

    if ( (d_time_for_last_loop * 1000 > 30) && (i_counter > 200) ) {
        cerr << " Took over 30 ms ..." << endl;
        cerr << " d_time_for_last_loop: " << d_time_for_last_loop * 1000.0;

        //      exit(0);
    }

    if ( (mClock.GetTime() - d_lasttime_loopstart) > 0.0011 ) {
        ossG << " [!! OVERTIME !!]";
    }

    ossG << endl;

    d_lasttime_loopstart = mClock.GetTime();

    timeTrack("A", b_debug);


    // / Update control Mode
    ctrlm_robot_prev = ctrlm_robot;
    ctrlm_robot      = ctrlm_robot_next;

    //  b_handonly_nullspace = b_handonly_nullspace_next;


    /**  ======================================
    * =   Process callback data (buttons)  =
    * ====================================== */

    if(b_enable_spmouse_buttons)
    if (joy_msg_.buttons.size() > 1) {
        // / right butt/on on up (only once per click)
        if ( joy_msg_.buttons.at(0) && !stdvec_prev_joy_buttons.at(0) && ( (mClock.GetTime() - d_lasttime_button0) > 0.2 ) ) {
            setContactLevel2(--i_contact_level, contact_levels);
            d_lasttime_button0 = mClock.GetTime();
        }

        // / left button on up (once per click)
        if ( joy_msg_.buttons.at(1) && !stdvec_prev_joy_buttons.at(1) && ( (mClock.GetTime() - d_lasttime_button1) > 0.2 ) ) {
            setContactLevel2(++i_contact_level, contact_levels);
            d_lasttime_button1 = mClock.GetTime();
        }

        // save last state for the detection of "ups"
        stdvec_prev_joy_buttons = joy_msg_.buttons;
    }

    timeTrack("A11: buttonsCallback", b_debug);


    /**  =================================
    * =          RESET VALUES         =
    * ================================= */

    v_additionnal_torques.Zero();
    v_additionnal_torque_friction.Zero();
    v_total_torque.Zero();
    v_contact_torque.Zero();
    v_virtual_contact_torques.Zero();
    v_torques_for_joint_error.Zero();


    /** Reset link contact status #resetcontact */
    for (int i_linkn = 0; i_linkn < NB_DOF_TOT; i_linkn++) {
        if ( b_init || ( (i_counter % 1) == 0 ) ) {
            int temp_size = ( (i_linkn < NB_DOF_ARM) ? (i_linkn) : (NB_DOF_ARM + (i_linkn - NB_DOF_ARM) % 4) ) + 1;

            // / ---------------
            // / VERSION JOINT

            // / Weight the weights with the inertia matrix !
            MathLib::Vector m_pid_weigths(temp_size);

            for (int j = 0; j < temp_size; j++) {
                m_pid_weigths(j) = m_jsim_tot(j, j);
            }

            m_pid_weigths += 0.1; // / add some weight (friction ...)


            // / -------------
            // / VERSION CART
            // / Goal is now to track a Cartesian position

            // Hardcoded here ... needs to be changed ...
            // maybe it should depend on "hand_only"
            list_links[i_linkn]->pid_controller_cart.Resize(3);
            list_links[i_linkn]->pid_controller_cart.SetKI( MathLib::Vector(3).Zero() );                                           // / I set to 0 ...
            list_links[i_linkn]->pid_controller_cart.SetKP(MathLib::Vector(3).One() * d_vc_cart_kp);
            list_links[i_linkn]->pid_controller_cart.SetKD(MathLib::Vector(3).One() * d_vc_cart_kd);                               // / try it ...

            list_links[i_linkn]->pid_controller_cart.SetBounds(MathLib::Vector(3).One() * (-5.0), MathLib::Vector(3).One() * 5.0); // important: avoid high forces!
            //      list_links[i_linkn]->pid_controller_cart.SetBounds(MathLib::Vector(3).One() * (-d_temp_3), MathLib::Vector(3).One() * d_temp_3); // important: avoid high forces!
            list_links[i_linkn]->pid_controller_cart.Start();
        }

        if (b_init) { // If init round, set the counter
            list_links[i_linkn]->i_contact_counter = 0;
            d_last_time_contact_down               = mClock.GetTime();
            d_last_time_for_contact_up             = mClock.GetTime();
        }


        // Decrease contact counter
        if (list_links[i_linkn]->i_contact_counter > 0) {
            list_links[i_linkn]->i_contact_counter--;
        }

        // If already 0, reset the contact
        if (list_links[i_linkn]->i_contact_counter == 0) {
            list_links[i_linkn]->contact_status = NO_CONTACT;
        }
    }

    timeTrack("A12: reset values", b_debug);

    if (i_counter == 10) {
        // Set current init endeff position
        v_cart_pos_tracked    = Eigen::Map<Eigen::Matrix<double, 3, 1> >(kdlf_endeff_pos_7_link.p.data, 3);
        v_cart_orient_tracked = Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> >(kdlf_endeff_pos_7_link.M.data, 3, 3);

        v_cart_pos_tracked_filtered    = v_cart_pos_tracked;
        v_cart_orient_tracked_filtered = v_cart_orient_tracked;


        myImpedanceController->SetStiffnessPosition_e(d_impedance_lin_stiff);
        myImpedanceController->SetDampingPosition_e(d_impedance_lin_damp);

        myImpedanceController->SetStiffnessOrientation_e(d_impedance_rot_stiff);
        myImpedanceController->SetDampingOrientation_e(d_impedance_rot_damp);

        //        myImpedanceController->EnableJointSpaceDamping(false);
        myImpedanceController->EnableJointSpaceDamping(true);
        myImpedanceController->SetJointSpaceDamping_e(d_joint_damping_coefficient);


        // v_curr_velocity should be 0ed for the fingers !!!
        MathLib::Vector v_curr_velocity_0_on_fingers = v_curr_velocity;
        v_curr_velocity_0_on_fingers.SetSubVector( NB_DOF_ARM, Vector(NB_DOF_HAND).Zero() );

        // NEWFILTER: reset the state to current ..
        cdd_desired_joint_pos_arm->SetState( v_curr_position.GetSubVector(0, NB_DOF_ARM), v_curr_velocity.GetSubVector(0, NB_DOF_ARM) );
        cdd_desired_joint_pos_hand->SetState( v_curr_position.GetSubVector(NB_DOF_ARM, NB_DOF_HAND), Vector(NB_DOF_HAND).Zero() );

        cdd_cartesian_position_motion->SetState( E2M_v(v_cart_endeff_pos) );

        cdd_cartesian_orientation_motion->SetState( E2M_v( v_cart_endeff_orient.eulerAngles(2, 1, 2) ) );

        MathLib::Vector vec(4);
        vec(0) = Eigen::Quaterniond(v_cart_endeff_orient).w();
        vec(1) = Eigen::Quaterniond(v_cart_endeff_orient).x();
        vec(2) = Eigen::Quaterniond(v_cart_endeff_orient).y();
        vec(3) = Eigen::Quaterniond(v_cart_endeff_orient).z();
        cdd_cartesian_orientationQ_motion->SetState(vec);


        ROS_INFO_STREAM("Reset the state of the cdd_desiredRadPos filter, i_counter: " << i_counter);


        // Set joint damping depending on joint inertia. (unused)
        //        double temp_joint_damping;

        //        for (int i = 0; i < NB_DOF_ARM; i++) {
        //            temp_joint_damping = m_jsim_tot(i, i) * d_joint_damping_coefficient;
        //            myImpedanceController->SetJointSpaceDamping_e(i, temp_joint_damping);
        //        }

        // Read current position
        v_ravin_joint_pos_tracked = M2E_v(v_curr_position);
    }

    // Init cartesian and joint PID controllers
    if (b_init) {
        pid_controller_cartesian.Resize(3);
        pid_controller_cartesian.SetKI( MathLib::Vector(3).Zero() );
        pid_controller_cartesian.SetKD(MathLib::Vector(3).One() * 10.0);
        pid_controller_cartesian.SetKP(MathLib::Vector(3).One() * 30.0);
        pid_controller_cartesian.SetBounds(MathLib::Vector(3).One() * (-20.0), MathLib::Vector(3).One() * 20.0);

        pid_controller_cartesian.Reset();
        pid_controller_cartesian.Start();

        pid_controller_joint.Resize(NB_DOF_TOT);
        pid_controller_joint.SetKI( MathLib::Vector(NB_DOF_TOT).Zero() );
        // the gains are set just afterwards
        //        pid_controller_joint.SetKD(MathLib::Vector(NB_DOF_TOT).One() * d_joint_position_kd);
        //        pid_controller_joint.SetKP(MathLib::Vector(NB_DOF_TOT).One() * d_joint_position_kp);
        pid_controller_joint.SetBounds(MathLib::Vector(NB_DOF_TOT).One() * (-20.0), MathLib::Vector(NB_DOF_TOT).One() * 20.0);
        pid_controller_joint.Reset();
        pid_controller_joint.Start();
    }

    if (b_init) {
        kps.Resize(NB_DOF_TOT);
        kds.Resize(NB_DOF_TOT);

        // Only Hand (static...)
        for (int i = 0; i < NB_DOF_HAND; i++) {
            kps(i + NB_DOF_ARM) = k_p[i];
            kds(i + NB_DOF_ARM) = k_d[i];
        }

        kps *= d_hand_kp_multiplier;
        kds *= d_hand_kd_multiplier;
    }

    // Update Kp and Jd gains at init and every 10 timesteps
    if ( (i_counter % 10 == 0) || b_init ) {
        // Update ARM gains depending on JSIM
        MathLib::Vector m_pid_weigths(NB_DOF_TOT);

        for (int j = 0; j < NB_DOF_TOT; j++) {
            m_pid_weigths(j)  = m_jsim_tot(j, j);
            m_pid_weigths(j) += 0.5; // Because of friction, need higher ...
        }

        // Only Arm
        for (int i = 0; i < NB_DOF_ARM; i++) {
            kps(i) = m_pid_weigths(i) * d_temp_5;
            kds(i) = m_pid_weigths(i) * d_temp_6;
        }


        // If update the gains ...
        if (!b_use_predetermined_pd_gains) {
            // Hand
            for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) {
                kps(i) = d_joint_position_kp;
                kds(i) = d_joint_position_kd;
            }
        }
        pid_controller_joint.SetKP(kps);
        pid_controller_joint.SetKD(kds);
    }

    // /////////////////////////////////////////

    timeTrack("A1: update kp/kds", b_debug);

    /**  =================================
    * =          ROBOT UPDATE         =
    * ================================= */

    // Do something with Kuka positions
    mSensorsGroup.ReadSensors();
    mActuatorsGroup.ReadActuators();

    // Read Robot Joint Angles and set as current position
    v_curr_position          = mSensorsGroup.GetJointAngles();
    v_curr_position_actuator =  mActuatorsGroup.GetJointAngles(); // Read Actuators position

    // ////////////////////////
    // Get AllegroHand's position
    v_curr_position.SetSubVector(NB_DOF_ARM, v_position_a);
    v_curr_position_actuator.SetSubVector(NB_DOF_ARM, v_position_a);

    // If connected to gazebo, get the robot positions and joint torques
    if (b_use_gazebo) {
        if(b_simulate_hand_only){
            for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) {
                v_curr_position[i] = v_curr_position_tempgaz[i-NB_DOF_ARM]; //Maybe a Pb here ...
            }
        }else{
        for (int i = 0; i < NB_DOF_TOT; i++) {
            v_curr_position[i] = v_curr_position_tempgaz[i];
        }
        }

        v_curr_position_actuator = v_curr_position;
    }

    v_est_joint_torques  = M2E_v( ( (LWRRobot *)mRobot )->GetEstimatedExternalJointTorques() );
    v_est_endeff_torques = M2E_v( ( (LWRRobot *)mRobot )->GetEstimatedExternalCartForces() );

    // Get robot's cart position
    MathLib::Vector3 lwr_cart_position;
    MathLib::Matrix3 lwr_cart_orientation;
    ( (LWRRobot *)mRobot )->GetMeasuredCartPose(lwr_cart_position, lwr_cart_orientation);

    lwr_measured_cart_position    = M2E_v(lwr_cart_position);
    lwr_measured_cart_orientation = M2E_m(lwr_cart_orientation);

    double d_ee_force_norm = v_est_endeff_torques.head(3).norm();
    //    ossGv << "External forces norm: " << d_ee_force_norm << endl;

    if (d_ee_force_norm > 10) {
        ossDebug << "Careful, End-effector torques are very high: " << d_ee_force_norm << endl;

        if (d_ee_force_norm > 30) {
            ctrlm_robot_next = GRAV_COMP;
            cerr << "End effector torques above 20N, going into gravity compensation mode" << endl;
        }
    }

    // The Goal is to update the information with the one that comes from the Allegro sensors
    // so that the Robot Kinematic chain can get updated, so I update both the Sensors and actuators with
    // new allegro Information

    mSensorsGroup.SetJointAngles(v_curr_position); // I just update the sensors with the filled position (+allegroHand)
    mSensorsGroup.WriteSensors();

    // TODO: Don't do that when working with the real robot ??? I need it for grav. comp but it seems it might set a position on the real robot ??
    // But I might need it for something else. no: the sensors actually do the thing, not the actuators.
    if (!b_control_real_robot) {
        mActuatorsGroup.SetJointAngles(v_curr_position_actuator); // I need to replace with (Actuators angles ...)
        mActuatorsGroup.WriteActuators();
    }

    mRobot->UpdateLinks();

    Compute_JSIM();

    if (b_init) {
        mRestPosition = v_curr_position; // save start position
    }

    timeTrack("B: robot update", b_debug);


    if(b_got_trajectory) {
        b_got_trajectory = false;
        d_time_start_trajectory = ros::Time::now().toSec();
        //        GetFirstTrajectoryPoint(planned_trajectory, desired_planned_joint_position);
        ossDebug << "desired_planned_joint_position: \n" << desired_planned_joint_position << endl;

    }

    // Time since last trajectory:

    //    ossGv << "desired_planned_joint_position: \n" << desired_planned_joint_position << endl;
    //    //    double d_time_since_plan_start = (ros::Time::now() - planned_trajectory.joint_trajectory.header.stamp).toSec();
    //    double d_time_since_plan_start = ros::Time::now().toSec() - d_time_start_trajectory;
    //    double time_scale = 0.3;
    //    if(b_follow_planner_joint_position)
    //        GetTrajectoryPointFromTime(planned_trajectory, d_time_since_plan_start, time_scale, desired_planned_joint_position);

    //    ossGv << "Time since last trajectory: \n" << d_time_since_plan_start << endl;


    /**  =================================
     * =       LOWPASS FILTERING       =
     * =================================  */

    v_curr_velocity = (v_curr_position - v_prev_position) / dt;
    timeTrack("B01: robot update", b_debug);

    // Filtering with CDDynamics
//    v_curr_position.Print("v_curr_position");
    cdd_positionFilter->SetTarget(v_curr_position);
    cdd_positionFilter->Update();
    cdd_positionFilter->GetState(curr_position_filtered_CDD, v_curr_velocity_filtered_CDD);
    timeTrack("B02: robot update", b_debug);

    cdd_velocityFilter->SetTarget(v_curr_velocity);
    cdd_velocityFilter->Update();
    cdd_velocityFilter->GetState(v_curr_velocity_filtered_CDD2, v_curr_acceleration_filtered_CDD2);
    timeTrack("B03: robot update", b_debug);

    v_curr_acceleration_filtered_CDD1 = (v_curr_velocity_filtered_CDD - v_prev_velocity_filtered_CDD) / dt;
    v_prev_velocity_filtered_CDD      = v_curr_velocity_filtered_CDD;

    timeTrack("B04: robot update", b_debug);

    // new filter with second degree ...
    v_current_velocity_filtered_2deg = (0.6 * v_current_velocity_filtered_2deg) +
            ( 0.198 * M2E_v(v_prev_velocity) ) +
            ( 0.198 * M2E_v(v_curr_velocity) );

    v_current_position_filtered_2deg = (0.6 * v_current_position_filtered_2deg) +
            ( 0.198 * M2E_v(v_prev_position) ) +
            ( 0.198 * M2E_v(v_curr_position) );
    timeTrack("B2: lowpass filtering", b_debug);

    /**  =====================================
     * =  UPDATE INV DYN FOR GRAVITY COMP  = (not used anymore because replaced with KDL), yes but where?
     * =====================================*/

    mInvDynamics.Update();
    mInvDynamics.GetTorques(v_gravComp_torque);

    timeTrack("C: inv. dynamics", b_debug);

    /**  =================================
    * =          READ TEKSCAN         =
    * ================================= */


    if (i_counter == 5) {
        b_reset_offset = true; // force an offset reset after a few counts
    }

    // Read and Parse data

    if (b_tekscan_con_success) {
        if ( readTekscanDual(b_reset_offset, tekpatch_set_1, tekpatch_set_2) ) { // check that we got new data
            if (i_counter > 215) {                                               // wait a bit at the beginning
                process_tactile_contact_tekscan(v_finger_contact_max_dof,
                                                i_nb_contacts,
                                                full_patch_list,
                                                stdvec_link_names_gazebo,
                                                mRobot,
                                                list_contacts,
                                                list_links,
                                                d_desired_normal_pressure);
            }
        }
    }

    // ------------------
    else if (b_use_gazebo_contact_data) {
        process_tactile_contact_gazebo2(v_finger_contact_max_dof,
                                        i_nb_contacts,
                                        stdvec_gazebo_contacts,
                                        stdmap_link_name_from_gazebo,
                                        mRobot,
                                        list_contacts,
                                        list_links,
                                        d_desired_normal_pressure);
    }


    ossG << "number of contacts: " << i_nb_contacts << endl;

    //  v_finger_contact_max_dof.Print("v_finger_contact_max_dof");


    /** -------------------------------------
     * Simulate some tekscan data
     * ----------------------------------------- */

    timeTrack("D", b_debug);

    /** -------------------------------------
     * Simulate some info from Ravin
     * ----------------------------------------- */

    if (b_use_ravins_method_for_vc && b_ravin_use_fake_mode_data) {
        // Force some values ...
        // index tip
        list_links[10]->i_side             = 0;
        list_links[10]->d_desired_pressure = 0.3;
        v_contact_on_link_allowed(10)      = 1.0;

        // medium tip
        list_links[14]->i_side             = 0;
        list_links[14]->d_desired_pressure = 0.3;
        v_contact_on_link_allowed(14)      = 1.0;

        // medium mid
        list_links[13]->i_side             = 0;
        list_links[13]->d_desired_pressure = 0.3;
        v_contact_on_link_allowed(13)      = 0.3;

        // last finger tip
        list_links[18]->i_side             = 0;
        list_links[18]->d_desired_pressure = 0.3;
        v_contact_on_link_allowed(18)      = 1.0;

        // index mid
        list_links[9]->i_side             = 0;
        list_links[9]->d_desired_pressure = 1.0;
        v_contact_on_link_allowed(9)      = 1.0;

        // thumb tip
        list_links[22]->i_side             = 1;
        list_links[22]->d_desired_pressure = 1.0;
        v_contact_on_link_allowed(22)      = 1.0;

        // thumb mid
        list_links[21]->i_side             = 1;
        list_links[21]->d_desired_pressure = 0.3;
        v_contact_on_link_allowed(21)      = 1.0;

        i_side_0_major = 14; // medium tip
        i_side_1_major = 22; // thumb mid
    }

    if (b_use_ravins_method_for_vc) {
        //        ossGv << "i_side_0_major: " << i_side_0_major << endl;
        //        ossGv << "i_side_1_major: " << i_side_1_major << endl;
    }

    //    v_contact_on_link_allowed.Print("v_contact_on_link_allowed");


    // / Create Task lists | unused ---
    TaskList tasklist( M2E_m(m_jsim_tot) );
    TaskList tasklist_hand_only;

    Mat jsim;
    if (b_use_identity_JSIM) {
        tasklist_hand_only._JSIM = Mat::Identity(NB_DOF_HAND, NB_DOF_HAND);
        jsim = Mat::Identity(NB_DOF_HAND, NB_DOF_HAND);

    } else {
        tasklist_hand_only._JSIM = M2E_m(m_jsim_tot).block(NB_DOF_ARM, NB_DOF_ARM, NB_DOF_HAND, NB_DOF_HAND);
        jsim = M2E_m(m_jsim_tot).block(NB_DOF_ARM, NB_DOF_ARM, NB_DOF_HAND, NB_DOF_HAND);
    }
    // -- unused


    //    OperationalSpaceControl ope_space_controller( M2E_m(m_jsim_tot) );
    //    cerr << "jsim: " << jsim << endl;
    OperationalSpaceControl ope_space_controller(jsim);
    OperationalSpaceControl ope_space_controller_ho(jsim);

    ope_space_controller.b_disableA1   = b_disableA1;
    ope_space_controller.b_disableAeq1 = b_disableAeq1;
    ope_space_controller.desired_contact_force = d_desired_normal_pressure;

//    ROS_INFO_STREAM_THROTTLE(2.0,"d_desired_normal_pressure: " << d_desired_normal_pressure );
    ope_space_controller.d_jsim_regularization = d_jsim_regularization;

    ope_space_controller_ho.b_disableA1   = b_disableA1;
    ope_space_controller_ho.b_disableAeq1 = b_disableAeq1;
    ope_space_controller_ho.desired_contact_force = d_desired_normal_pressure;
    ope_space_controller_ho.d_jsim_regularization = d_jsim_regularization;





    /**  ===================================================
    * =       Check distance to joint limits             =
    * =================================================== */


    // move those to header
    Vec v_dist_2_limits(NB_DOF_TOT);
    Vec v_dist_2_center(NB_DOF_TOT);
    Vec v_dist_2_center_percent(NB_DOF_TOT);

    distanceToJointLimits(M2E_v(v_curr_position), stdv_limits, v_dist_2_limits, v_dist_2_center, v_dist_2_center_percent);
    string s_joint_limits = getJointLimitsString(v_dist_2_center_percent, 36);

    ossDebug << s_joint_limits;

    double joint_limit_threshold = 0.5; // 30 degrees ! that's a lot ...

    for (int i = 0; i < NB_DOF_TOT; i++) {
        Mat joint_jacobian = Mat::Zero(1, NB_DOF_TOT);
        joint_jacobian(i) = 1.0;

        // If reaching the top joint limit: add a max force
        if ( (v_dist_2_limits(i) > 0.0) && (v_dist_2_limits(i) < joint_limit_threshold) ) {
            if (b_avoid_joint_limits) {
                tasklist.addTask( InegalityTask(joint_jacobian,
                                                0,
                                                Vec::Zero(NB_DOF_TOT),
                                                0,
                                                "Reached max joint limit on joint " + std::to_string(i), d_task_weight_joint_limit) );
                ope_space_controller.addJointLimitReached(i, true);
            }
        } else if ( (v_dist_2_limits(i) < 0.0) && (v_dist_2_limits(i) > -joint_limit_threshold) ) {
            // If reaching the bottom joint limit, add a min force, so a max force with inverted jacobian
            if (b_avoid_joint_limits) {
                tasklist.addTask( InegalityTask(-joint_jacobian, // sure about the direction ?
                                                0,
                                                Vec::Zero(NB_DOF_TOT),
                                                0,
                                                "Reached min joint limit on joint " + std::to_string(i),
                                                d_task_weight_joint_limit) );
                ope_space_controller.addJointLimitReached(i, false);
            }
        }
    }

    timeTrack("D1", b_debug);

    /**  ===================================================
    * =   COMPUTE TASK TO REACH CENTERED JOINT POSITION   =
    * =================================================== */

    // The current version is too fast on the real robot.
    Vec torque_centering = Vec::Zero(NB_DOF_TOT);

    double d_min_jointlimit_distance_percent_finger = 5.0;
    double d_max_torque_finger                      = 0.1;

    for (int i = 0; i < NB_DOF_ARM; i++) {
        torque_centering(i) = getLimitAvoidanceTorque(v_dist_2_center_percent(i), d_min_jointlimit_distance_percent_arm, d_max_torque_arm_centering);
    }

    for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) {
        torque_centering(i) = getLimitAvoidanceTorque(v_dist_2_center_percent(i), d_min_jointlimit_distance_percent_finger, d_max_torque_finger);
    }

    if (b_use_joint_centering) {
        //        tasklist.addTask( EgalityTask(center_task_jacobian.transpose(),
        //                                      2,
        //                                      torque_centering,
        //                                      "Joint centering", d_task_weight_joint_centering) );
        ope_space_controller.addObjective(torque_centering, d_task_weight_joint_centering, "Joint centering");

        //                ossDebug << "torque_centering: \n" << torque_centering << endl;
    }

    /**  ===================================================
    * =    BUILD A TABLE OF JOINTS WITH CONTACT INFO    =
    * =================================================== */

    v_joint_contact_info.Zero();

    for (int i = 0; i < i_nb_contacts; i++) {
        int i_hand_joint;

        if (list_contacts[i]->i_dof_n > NB_DOF_ARM - 1) {                                      // Only if not a contact on the arm
            i_hand_joint                          = list_contacts[i]->i_dof_n - NB_DOF_ARM;
            v_joint_contact_info(i_hand_joint, 0) = 1.0;                                       /* 0: contact or not ... */
            v_joint_contact_info(i_hand_joint, 1) = list_contacts[i]->d_pressure;              /* 0: pressure */
            v_joint_contact_info(i_hand_joint, 2) = list_contacts[i]->v_position_from_link[0]; /* 0: pos0 */
            v_joint_contact_info(i_hand_joint, 3) = list_contacts[i]->v_position_from_link[1]; /* 0: pos1 */
            v_joint_contact_info(i_hand_joint, 4) = list_contacts[i]->v_position_from_link[2]; /* 0: pos2 */
            v_joint_contact_info(i_hand_joint, 5) = i;                                         /* 0: contactID */
        }
    }

    timeTrack("D2", b_debug);


    /**   ===================================================
     * =          Find average normal of contact         =
     * =================================================== */

    int i_nb_allowed_contacts_status = 0;

    Eigen::Vector3d v3_choosen_contact_normal = compute_main_normal_links(list_links, i_nb_allowed_contacts_status, true);

    // Filter the normal contact
    if (i_nb_allowed_contacts_status == 0) { // set the filter value to current orientation if no contact
        MathLib::Vector normal(3);
        normal(0) = v_cart_endeff_orient.col(0) (0);
        normal(1) = v_cart_endeff_orient.col(0) (1);
        normal(2) = v_cart_endeff_orient.col(0) (2);
        cdd_contact_normal_filter->SetState(normal);
        cdd_contact_normal_filter->SetTarget(normal);
        //        ossG << "no allowed contacts on" << endl;
    } else {
        //        ossG << "allowed contacts on" << endl;
        cdd_contact_normal_filter->SetTarget( MathLib::Vector(v3_choosen_contact_normal.data(), 3) );
    }

    cdd_contact_normal_filter->Update();
    MathLib::Vector contact_normal_filtered_math(3);
    cdd_contact_normal_filter->GetState(contact_normal_filtered_math);
    contact_normal_filtered = Eigen::Map<Eigen::Vector3d>( contact_normal_filtered_math.Array() );

    timeTrack("D3", b_debug);

    /**   ===================================================
     * =          Use average normal of contact         =
     * =================================================== */

    // If there is a contact, use this frame as the desired orientation
    // Otherwise, the orientation is controlled by the spaceMouse
    normal_Frame = FrameFromNormal(contact_normal_filtered, v_cart_endeff_orient.col(1), 0); // enable to give a 2nd normal information


    // Compute direction of motion towards "final" goal of exploration
    Eigen::Vector3d e_desired_direction, e_desired_direction_projected, e_desired_position;
    double tracking_gain       = 0.06;
    double d_slowing_threshold = 0.05;

    e_desired_direction = e_final_pos_exploration - v_cart_endeff_pos;

    // The desired direction is projected on the contact tangents (remove normal component)
    e_desired_direction_projected = e_desired_direction - ( e_desired_direction.dot(v3_choosen_contact_normal) ) * v3_choosen_contact_normal;


    if (e_desired_direction_projected.norm() > d_slowing_threshold) { // /TODO: change test with  e_desired_direction_projected.norm()
        e_desired_position = v_cart_endeff_pos + e_desired_direction_projected.normalized() * tracking_gain
                + v3_choosen_contact_normal * tracking_gain / 2.0;
    } else {
        e_desired_position = v_cart_endeff_pos + e_desired_direction_projected * tracking_gain / d_slowing_threshold;
    }


    // Decide on the cartesian impedance target
    if (i_nb_allowed_contacts_status > 0) {          // if contacts
        if (b_auto_pos_tracking) {
            v_cart_pos_tracked = e_desired_position; // only if tracking position
        }

        if (b_auto_orientation) {
            v_cart_orient_tracked = normal_Frame;
        }
    } else {
        if (b_auto_pos_tracking) { // if bvel activated
            if (b_auto_orientation) {
                v_cart_orient_tracked = FrameFromNormal(e_desired_direction.normalized(), v_cart_endeff_orient.col(1), 0);
            }

            // Check angle
            double orientation_error = acos( e_desired_direction.normalized().dot( v_cart_endeff_orient.col(0) ) ) * 180.0 / PI;

            if (orientation_error < 30.0) {
                v_cart_pos_tracked = v_cart_endeff_pos + tracking_gain *e_desired_direction. normalized();
            } else {
                v_cart_pos_tracked = v_cart_endeff_pos;
            }
        }
    }


    // If reached the target position, go to next target if the mode is activated
    //    double d_target_reached_threshold = 0.04;

    if ( (e_desired_direction_projected.norm() < d_target_reached_threshold) && b_auto_next_point ) {
        if ( (i_curr_despos_i + 1) < stdvec_des_cart_positions.size() ) {
            i_curr_despos_i++;
        } else {
            i_curr_despos_i = 0;
        }

        e_final_pos_exploration = stdvec_des_cart_positions[i_curr_despos_i];
    }


    /** ===================================================
     * =          Find max intensity of contacts         =
     * =================================================== */

    double max_intensity                        = 0.0;
    int    max_intensity_counter                = 20;
    std::string s_name_of_max_intensity_contact = "";

    for (int i = 0; i < i_nb_contacts; i++) {
        if (list_contacts[i]->d_pressure > max_intensity) {
            max_intensity_counter           = 2; // 10 for streaming
            max_intensity                   = list_contacts[i]->d_pressure;
            s_name_of_max_intensity_contact = list_contacts[i]->s_link_name_urdf;
        }
    }

    max_intensity_counter--;

    if (max_intensity_counter == 0) {
        max_intensity = 0.0;

        // reset the max_intensity info..
    }

    v_max_intensity(0) = max_intensity;
    cdd_max_intensity->SetTarget(v_max_intensity);
    cdd_max_intensity->Update();
    cdd_max_intensity->GetState(v_max_intensity_filtered);

    ossG << "Contact with max intensity: " << max_intensity << " on " << s_name_of_max_intensity_contact << endl;

    timeTrack("D4", b_debug);

    /**  ====================================================
    * =              DETERMINE CONTROL MODE              =
    * ==================================================== */


    // Test whether finger should be opened or not (open signal from EMG)
    ComputeFingerOpenCriteria(E2M_v(v_hand_explo_posture), d_angle_sum_max_limit, b_fingers_should_open);
    //    ComputeFingerOpenCriteria(v_desired_hand_pos_filtered, d_angle_sum_max_limit, b_fingers_should_open);
    // Reset finger filter if the state has changed ...
    for (int finger = 0; finger < NB_FINGERS;finger++){
        MathLib::Vector state_vector_corrected;
        MathLib::Vector state_velocity_corrected;
        cdd_desired_joint_pos_hand->GetState(state_vector_corrected,state_velocity_corrected);

        if(b_fingers_should_open_old[finger]==0){
            if(b_fingers_should_open[finger]==1){
//                ROS_INFO_STREAM("finger" << finger << " changed from 0 to 1");
                // reset the filter for that finger ... (does it work ???)
                // a) create vector
                state_vector_corrected.SetSubVector(finger*4,v_curr_position.GetSubVector(NB_DOF_ARM+finger*4, NB_DOF_FINGER));
                state_velocity_corrected.SetSubVector(finger*4,v_curr_velocity.GetSubVector(NB_DOF_ARM+finger*4, NB_DOF_FINGER)); // fix

            }
        }
        cdd_desired_joint_pos_hand->SetState(state_vector_corrected, state_velocity_corrected ); // not sure about the velocity.. may cause trouble ...
        // copy old values
        b_fingers_should_open_old[finger] = b_fingers_should_open[finger];
    }

    //    define_control_mode(jointCtrlMode, v_contact_on_link_allowed, v_finger_contact_max_dof, finger_enabled, i_nb_contacts, b_ravin_close_mode, b_close_allowed);
    //    define_control_mode_emg(jointCtrlMode, v_contact_on_link_allowed, v_finger_contact_max_dof, finger_enabled,v_desired_hand_pos_filtered);
    define_control_mode_emg(jointCtrlMode, v_contact_on_link_allowed, v_finger_contact_max_dof, finger_enabled,b_fingers_should_open);

    // PB: if using PD control in nullspace, normal pd control does not work anymore... -> change it in a different way ?

    // Debug
    for (int i = 0; i < NB_DOF_TOT; i++) {
        // ossGv << "jointCtrlMode[:" << i << "]:\t" <<jointCtrlMode[i] << endl;
        // ossDebug << "jointCtrlMode[:" << i << "]:\t" <<jointCtrlMode[i] << endl;
    }


    /**   ===================================================
     * =         RAVIN's experiments STATE MACHINE       =
     * =================================================== */

    // Security: set them to current position
    // They are set to a correct value later on. (depending on ravin . task ...)
    if (i_counter == 200) {
        v_cart_pos_tracked_ravin    = v_cart_endeff_pos;
        v_cart_orient_tracked_ravin = v_cart_endeff_orient;
    }

    /*
    // Stuff with Ravin's controller
    if(0){ // disabled for working with EMG, maybe need it later
        // REACH MODE
        // Ravin's controller: during the reaching motion (triggered by a callback)
        if (b_ravin_jointpos_reach) {
            // During reaching, all joints in position_PID
            for (int i = 0; i < NB_DOF_TOT; i++) {
                jointCtrlMode[i] = POSITION_PID;
            }

            ossGv << "Last time since Ravin's trajectory being sent: " << (mClock.GetTime() - d_last_time_trajectory) << endl;

            // TRANSITION to CLOSING
            if (b_ravins_mode) {
                // Enough time to transition since last trajectory sent
                if (mClock.GetTime() - d_last_time_trajectory > d_ravin_min_time_for_reach_finished) {
                    // Check arm's position if trying to control it
                    if ( (!b_control_real_robot && !b_use_gazebo) || ( ( v_ravin_joint_pos_tracked - M2E_v(v_curr_position) ).head(7).norm() < 0.03 ) ) {
                        // Check for hand's position
                        Vec v_grasp_error_hand;
                        v_grasp_error_hand      = ( v_ravin_joint_pos_tracked - M2E_v(v_curr_position) ).tail(NB_DOF_HAND);
                        v_grasp_error_hand(4)  *= 0.5;
                        v_grasp_error_hand(8)  *= 0.5;
                        v_grasp_error_hand(12) *= 0.5;

                        if(b_disable_index){
//                          v_grasp_error_hand.SetSubVector(0,MathLib::Vector(4));
                          v_grasp_error_hand.head(4).setZero();
                        }
                        ossGv << "v_grasp_error_hand norm: " << v_grasp_error_hand.norm() << endl;

                        if ( (v_grasp_error_hand.norm() < d_ravin_hand_error_norm_threshold_reach) || b_ravin_bypass_hand_error_check_reach ) { // Don't check hand (debug)
                            if (v_curr_velocity_filtered_CDD.Norm() < d_ravin_hand_velocity_threshold_reach) {
                                PublishState("finished reach");


                                // TRANSITION to CLOSE: Switch if allowed in config file or from Ravin's signal
                                if (b_ravin_auto_close_switch || b_close_signal) {
                                    // Switch to Closing mode, and put robot in cart impedance mode
                                    b_ravin_jointpos_reach = false;
                                    b_ravin_close_mode     = true;
                                    d_last_time_trajectory = mClock.GetTime();

                                    b_ravin_get_a_direction = true;
                                    i_no_torque             = 10; // useless?
                                    // reset other signals
                                    b_task_mode_signal = false;
                                    b_task_rotation    = false;
                                    b_task_hammering   = false;
                                    b_close_signal     = false;


                                    // Set desired tracking reference to current position (only in simulation, could be done on the robot too ...)
                                    //                b_cart_command_ready        = true;
                                    v_cart_pos_tracked_ravin    = v_cart_endeff_pos;
                                    v_cart_orient_tracked_ravin = v_cart_endeff_orient;

                                    lwr_cart_position_init_e    = v_cart_endeff_pos;
                                    lwr_cart_orientation_init_e = v_cart_endeff_orient;

                                    // Set the tracked position as here (when using spacemouse on top)
                                    v_cart_pos_tracked    = v_cart_endeff_pos;
                                    v_cart_orient_tracked = v_cart_endeff_orient;


                                    // Copy for the tasks...
                                    task_hammering->lwr_cart_position_init_e    = lwr_cart_position_init_e;
                                    task_hammering->lwr_cart_orientation_init_e = lwr_cart_orientation_init_e;

                                    task_rotation->lwr_cart_position_init_e    = lwr_cart_position_init_e;
                                    task_rotation->lwr_cart_orientation_init_e = lwr_cart_orientation_init_e;


                                    lwr_des_cart_position_e    = lwr_cart_position_init_e;
                                    lwr_des_cart_orientation_e = lwr_cart_orientation_init_e;


                                    GetConsole()->Print("Reached desired joint position");
                                    PublishState("closing");
                                }
                            }
                        }
                    }
                }
            }
        }


        // CLOSE MODE
        if (b_ravin_close_mode) {
            if (ctrlm_robot != TORQUE) ctrlm_robot_next = TORQUE;


            // TRANSITION to GRASP MODE
            if (mClock.GetTime() - d_last_time_trajectory > d_ravin_time_auto_mini_tighten_switch) {
                if ( (v_curr_velocity_filtered_CDD.GetSubVector(NB_DOF_ARM, NB_DOF_HAND).Norm() < 0.1) || 1 ) { // force allow transition .. (IGNORE VELOCITY)
                    GetConsole()->Print("Reached enough compliance");
                    PublishState("finished closing");


                    if (b_ravin_allow_tighten_switch && b_tighten_signal) {
                        // Save current joint positions, and add 5 degrees for most axes
                        v_ravin_joint_pos_grasped = M2E_v(v_curr_position);

                        // Increase this by adding a few degrees on the closing joints
                        for (int i = 0; i < NB_DOF_HAND; i++) {
                            int i_hand = i + NB_DOF_ARM;

                            if ( (i % 4 == 0) || ( i == (NB_DOF_FINGER * 3 + 1) ) ) {} else {
                                v_ravin_joint_pos_grasped(i_hand) += 30.0 * M_PI / 180.0;
                                ROS_WARN_STREAM("INcrease grasp position for joint " << i_hand);
                            }
                        }

                        // Reset motion generator filters (why here ?)
                        cdd_desired_joint_pos_arm->SetState( v_curr_position.GetSubVector(0, NB_DOF_ARM), v_curr_velocity.GetSubVector(0, NB_DOF_ARM) );
                        cdd_desired_joint_pos_hand->SetState( v_curr_position.GetSubVector(NB_DOF_ARM, NB_DOF_HAND), Vector(NB_DOF_HAND).Zero() );

                        pid_controller_joint.Reset();
                        i_no_torque = 5;


                        // Remove all other modes
                        b_ravin_jointpos_reach = false;
                        b_ravin_close_mode     = false;
                        b_ravin_grasped_mode   = true;
                        PublishState("grasping");
                    }
                }
            }
        }

        // GRASP MODE
        if (b_ravin_grasped_mode) {
            for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) jointCtrlMode[i] = POSITION_PID;

            if (ctrlm_robot != TORQUE) ctrlm_robot_next = TORQUE;

            //            // TRANSITION to TASK
            //            if (b_ravin_task_mode_allowed && b_task_mode_signal) {
            //                b_task_mode_signal = false;

            //                b_ravin_task_mode    = true;
            //                b_ravin_grasped_mode = false;
            //                b_finished_task      = false;

            //                b_task_rotation  = false;
            //                b_task_hammering = false;

            //                task_hammering->d_time_start = ros::Time::now();
            //                task_rotation->d_time_start  = ros::Time::now();

            //                time_got_task_signal = ros::Time::now();
            //                ossGv << "Setting task signal time: " << endl;
            //                PublishState("Task mode");
            //            }
        }


        //        // TASK MODE
        //        if (b_ravin_task_mode) {
        //            for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) jointCtrlMode[i] = POSITION_PID;
        //        }


        // Depending on which version I want ...
        //        if (b_ravin_task_mode) {
        //            // Do it with the functions
        //            if (b_task_hammering) b_finished_task = task_hammering->ExecuteTask(v_cart_pos_tracked_ravin, v_cart_orient_tracked_ravin);

        //            if (b_task_rotation) b_finished_task = task_rotation->ExecuteTask(v_cart_pos_tracked_ravin, v_cart_orient_tracked_ravin);

        //            //    }
        //        } else {
        //            // for unscrewing
        //            d_ravin_task_rotation = 0.0;
        //        }
    }

    if (b_use_ravins_method_for_vc && 1) {
//        ossGv << "b_ravin_jointpos_reach: " << b_ravin_jointpos_reach  << endl;
//        ossGv << "b_ravin_close_mode: " << b_ravin_close_mode  << endl;
//        ossGv << "b_close_allowed: " << b_close_allowed  << endl;
//        ossGv << "b_ravins_mode: " << b_ravins_mode  << endl;

        //    ossGv << "b_ravin_grasped_mode: " << b_ravin_grasped_mode  << endl;
        //    ossGv << "b_ravin_task_mode: " << b_ravin_task_mode  << endl;
        //    ossGv << "b_task_hammering: " << b_task_hammering  << endl;
        //    ossGv << "b_task_rotation: " << b_task_rotation  << endl;
        //    ossGv << "b_ravin_cart_reach: " << b_ravin_cart_reach  << endl;
    }

    */


    /**  ====================================================
    * =            BUILD VIRTUAL CONTACT LIST            =
    * ==================================================== */

    timeTrack("E", b_debug);


    // / Version from KDL: ok, but the normal influences the direction of the virtual force, which is not really desired:
    // / Either change the normal towards the existing contact here (or some sort of projection)
    // / or leave the normal "as is", but edit later the direction of the virtual force

    /** Only two links possible per finger: joints 1 and 2 (links 0 and 1)
     * This works only for the hand, what happens if we count the whole arm ?
     */

//    build_virtual_contact_list(list_links, v_contact_on_link_allowed, finger_enabled, jointCtrlMode);
    build_virtual_contact_list(list_links, v_contact_on_link_allowed, finger_enabled, jointCtrlMode, b_fingers_should_open);


    for (int i = 0; i < NB_DOF_TOT; i++) {
        ossDebug << "jointCtrlMode[" << i << "]: " << jointCtrlMode[i] << endl;
    }


    timeTrack("E1", b_debug);


    /**  ============================================
     * =   UPDATE CONTACT JACOBIANS (KDL)   =
     * ============================================= */


    //    UpdateJacobianAndFk(list_contacts, i_nb_contacts, false, true);
    UpdateJacobianAndFk(list_contacts, i_nb_contacts, false, false);
    timeTrack("E11", b_debug);

    // For the links, do not rotate the contacts: when we need it, we do the rotation later.
    // The issue is that I don't include the orientation of the contact in the rotation, so I can change it for virtual contacts for instance.
    UpdateJacobianAndFk(list_links, NB_DOF_TOT, false, false);

    timeTrack("E2", b_debug);

    /**  =================================
    * =  COMPUTE TORQUES FOR CONTACT  =
    * ================================= */


    // / This has been redone so that I can have matching jacobians and torques for the hierarchical nullspace projections

//    for(int finger=0;finger<4;finger++){
//        ROS_INFO_STREAM("b_fingers_should_open:[" << finger << "]:" << b_fingers_should_open[finger]);
//    }

    // Compute torque for each link
    for (int i_c = 0; i_c < NB_DOF_TOT; i_c++) {
//        cout << "### i_c:" << i_c << endl;
        if (list_links[i_c]->contact_status == CONTACT) {
            cout << "there is contact" << endl;
            // check if finger should open or not
            cout << "b_fingers_should_open[(i_c - NB_DOF_ARM) % 4]:" << b_fingers_should_open[(i_c - NB_DOF_ARM) / 4] << endl;
            if(i_c<NB_DOF_ARM ||!b_fingers_should_open[(i_c - NB_DOF_ARM) / 4]){
                cout << "inside the loop" << endl;


            Mat jacobian_short_e = jacobianSpe1RowPerContactFromLinkID_e(list_links, i_c);
            Mat jacobian_23_e    = get_full_vector_from_short_e(jacobian_short_e, list_links[i_c]->i_finger_n); // / DEBUG HERE: finger is not good !

            //            ossGv << "des pressure: " << list_links[i_c]->d_desired_pressure << endl;

//            Vec contact_torque_e = list_links[i_c]->d_desired_pressure * d_ravin_patch_pressure_mutliplier *  jacobian_23_e.transpose();
            Vec contact_torque_e = d_desired_normal_pressure * d_ravin_patch_pressure_mutliplier *  jacobian_23_e.transpose();
            v_contact_torque += MathLib::Vector( contact_torque_e.data(), contact_torque_e.rows() );

            // Add contact task
            if ( v_contact_on_link_allowed(i_c) ) {
                tasklist.addTask( EgalityTask( jacobian_23_e, 0, contact_torque_e, "Desired contact on link " + std::to_string(i_c) ) );
                ope_space_controller.addContact( jacobian_23_e, true, "Desired contact on link " + std::to_string(i_c) );
            } else {
                tasklist.addTask( InegalityTask( jacobian_23_e, 0, Vec::Zero(NB_DOF_TOT), 0.0, "Undesired contact on link " + std::to_string(i_c) ) );
                ope_space_controller.addContact( jacobian_23_e, false, "Undesired contact on link " + std::to_string(i_c) );
            }

//            ROS_INFO_STREAM_THROTTLE(1.0,"Contact on link"+ std::to_string(i_c));
            // Short contact task
            if ( v_contact_on_link_allowed(i_c) ) {
//                ROS_INFO_STREAM_THROTTLE(1.0,"DEsired Contact on link"+ std::to_string(i_c));
                tasklist_hand_only.addTask( EgalityTask( jacobian_23_e.rightCols(NB_DOF_HAND), 0, contact_torque_e.tail(NB_DOF_HAND), "Desired contact on link " + std::to_string(i_c) ) );
                ope_space_controller_ho.addContact( jacobian_23_e.rightCols(NB_DOF_HAND), true, "Desired contact on link " + std::to_string(i_c) );

            } else {
                if (b_protect_undesired_contacts) {
//                    ROS_INFO_STREAM_THROTTLE(1.0,"UNdesired contact on link"+ std::to_string(i_c));
                    tasklist_hand_only.addTask( InegalityTask( jacobian_23_e.rightCols(NB_DOF_HAND), 0, Vec::Zero(NB_DOF_HAND), 0.0, "Undesired contact on link " + std::to_string(i_c) ) );
                    ope_space_controller_ho.addContact( jacobian_23_e.rightCols(NB_DOF_HAND), false, "Undesired contact on link " + std::to_string(i_c) );

                }
            }
        }
    }
    }


    timeTrack("F", b_debug);

    //    ossGv << "v_contact_torque: \n" << M2E_v(v_contact_torque) << endl;

    //    v_contact_torque.Print("v_contact_torque: ");


    /**  =================================================
    * =  COMPUTE VIRTUAL TORQUES FOR VIRTUAL CONTACT  =
    * ================================================= */


    if (b_use_ravins_method_for_vc && (b_ravin_close_mode || b_close_allowed) && b_ravin_get_a_direction) {
        // When was last time these were updated ??? with which values ?
        v3_dir_p0_to_p1.Set( (list_links[i_side_1_major]->_kdl_frame.p - list_links[i_side_0_major]->_kdl_frame.p).data );


        // We have recorded the direction, don't do it anymore if it's small ...
        if (v3_dir_p0_to_p1.Norm() > 0.01) {
            GetConsole()->Print("Got the direction value: ");

            b_ravin_get_a_direction = false;
            v3_dir_p0_to_p1.Normalize();
        } else {
            v3_dir_p0_to_p1[0] = 1.0; // Is this necessary ???
        }
    }

    //  ossGv << "v3_dir_p0_to_p1: " << M2E_v(v3_dir_p0_to_p1) << endl;


    Vector v_virtual_contact_torque_new;
    v_virtual_contact_torque_new.Resize(NB_DOF_TOT);
    v_virtual_contact_torque_new.Zero();


    // Set desired forces in Hand and Link Frames
    if (b_use_virtual_contact_torques) // added a test if working with ravin's stuff !

        // NEEDED:
        // list_links (everywhere)
        // jointCtrlMode (just to decide at the beg)
        // ossGv (debug)
        // b_use_ravins_method_for_vc
        // b_temp_1 (b_parallel_direction)
        // b_use_vc_normalinfo
        // v3_dir_p0_to_p1
        // b_use_virtual_contact_torques
        // d_ravin_finger_orient_stiffness
        // getfullVectorFromShort
        // b_temp_2
        // tasklist

        /*
         * a) Get closest point / direction of integration
         * INPUT: v3_dir_p0_to_p1, method for computing, list_links other contacts
         * OUTPUT: v3_virt_veldir_in_root_frame
         *
         *
         * b) integrate cart velocity
         *  APPLY ON values
         *
         *
         * c) imp control: PD -> force
         * INPUT: cartesian target and current
         * OUTPUT: v_cart_force_pid
         *
         *
         * d) apply force : jac transpose ...
         * INPUT: jacobian, force
         * OUTPUT: torques, 1-row jacobian
         *
         *
         * e) add to task list
         *
         * */


        for (int i_vc = 0; i_vc < NB_DOF_TOT; i_vc++) {
            // Check that it's a virtual contact and that the control mode is set (it should be active ...)
            if ( (list_links[i_vc]->contact_status == VIRTUAL_CONTACT) && (jointCtrlMode[i_vc] == TORQUE) ) {
                //        ossG << i_vc  << "\t " << list_links[i_vc]->s_link_name_urdf << endl;

                Vector3d v3_virt_vel_in_root_frame;
                Vector3d v3_virt_veldir_in_root_frame;

                // Check in a different way ...
                if ( (i_nb_contacts > 0) || b_use_ravins_method_for_vc ) {
                    //                    timeTrack("G00", true);

                    // Using Ravin's function:
                    if (b_use_ravins_method_for_vc) {
                        v3_virt_veldir_in_root_frame = getDirectionOfClosingRavin(list_links, i_vc, i_side_1_major, i_side_0_major, M2E_v(v3_dir_p0_to_p1), b_ravin_parrallel_closing);
                    } else {
                        //            v3_virt_veldir_in_root_frame = getDirectionOfClosingClosest(list_links, i_vc, b_use_vc_normalinfo);
                        v3_virt_veldir_in_root_frame = getDirectionOfClosingClosestContinuous(list_links, i_vc, d_use_vc_normalinfo);
                    }

                    // Third method: use simply the normal of the virtual contact.

                    // Apply desired virtual velocity
                    v3_virt_vel_in_root_frame = v3_virt_veldir_in_root_frame * d_desired_virtual_normal_velocity;
                    Vector3d v3_proj_dir = v3_virt_vel_in_root_frame / v3_virt_vel_in_root_frame.norm();


                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                    //                    timeTrack("G01", true);

                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    // Integrate the CARTESIAN Velocity
                    if (b_use_virtual_contact_torques) {
                        // If 0, reset to current cartesian position (it's a signal)
                        if (list_links[i_vc]->v_desired_cart_position.norm() < 0.0001) {
                            tf::vectorKDLToEigen(list_links[i_vc]->_kdl_frame.p, list_links[i_vc]->v_desired_cart_position);
                        }

                        list_links[i_vc]->v_desired_cart_position += v3_virt_vel_in_root_frame * dt * 0.01;

                        // If the desired cart position is too far from a predefined distance, bring it back
                        Vector3d v_dist_to_target = list_links[i_vc]->v_desired_cart_position - Vector3d(list_links[i_vc]->_kdl_frame.p.data);

                        // Project the target point on the current desired direction (and only positive) (direction and min bound)
                        list_links[i_vc]->v_desired_cart_position = Vector3d(list_links[i_vc]->_kdl_frame.p.data) + v3_proj_dir * ( max(v_dist_to_target.dot(v3_proj_dir), 0.0) );


                        // Set the desired cart position at the right distance from current position (max bound)
                        if (v_dist_to_target.norm() > d_vc_cart_max_gap) {
                            list_links[i_vc]->v_desired_cart_position = Vector3d(list_links[i_vc]->_kdl_frame.p.data) + v_dist_to_target / v_dist_to_target.norm() * d_vc_cart_max_gap;
                        }
                    }

                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    // PID to reach the desired cartesian position (basically impedance control)
                    list_links[i_vc]->pid_controller_cart.SetTarget( E2M_v(list_links[i_vc]->v_desired_cart_position) );
                    list_links[i_vc]->pid_controller_cart.SetInput( MathLib::Vector(list_links[i_vc]->_kdl_frame.p.data, 3) );
                    list_links[i_vc]->pid_controller_cart.Update(0.001);

                    MathLib::Vector v_cart_force_pid;

                    //              MathLib::Vector v_cart_force_pid_proj_root;
                    list_links[i_vc]->pid_controller_cart.GetOutput(v_cart_force_pid);

                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    // Project this force in the current direction towards the closest contact:

                    //          Vector3d v3_proj_dir = v3_virt_vel_in_root_frame / v3_virt_vel_in_root_frame.norm();
                    // not necessary anymore
                    //          v_cart_force_pid_proj_root = E2M_v( v3_proj_dir * ( v_cart_force_pid.Dot( E2M_v(v3_proj_dir) ) ) );

                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                    //                    timeTrack("G02", true);

                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    // Add the desired rotation direction (UNUSED !)
                    Vector3d v1, v2;
                    v1 = M2E_v( list_links[i_vc]->normalInBaseFrame() );

                    //                    timeTrack("G027", true);

                    // v2: either the same as main patch, or directly this patch
                    if (!b_ravin_parrallel_closing) {
                        int i_opposing_contact;

                        if (list_links[i_vc]->i_side == 0) {
                            i_opposing_contact = i_side_1_major;
                        } else {
                            i_opposing_contact = i_side_0_major;
                        }

                        tf::vectorKDLToEigen( (list_links[i_opposing_contact]->_kdl_frame.p - list_links[i_vc]->_kdl_frame.p), v2 );
                    } else {
                        v2 = M2E_v(v3_dir_p0_to_p1);


                        if (list_links[i_vc]->i_side == 1) v2 = -v2;
                        v2 /= v2.norm();
                    }

                    //                    timeTrack("G028", true);

                    // Angle of rotation:
                    double angle = acos( v1.dot(v2) );

                    // Axis of rotation:
                    Vector3d v3 = v1.cross(v2);

                    if (v3.norm() != 0) {
                        v3 /= v3.norm();
                    } else {
                        // ignore for now
                    }

                    //                    timeTrack("G029", true);

                    // The combination of error and axis-angle should be enough at the end.
                    // I need an orientation stiffness in that direction.
                    Vec v_cart_orient_torque_in_cart = v3 * angle * d_ravin_finger_orient_stiffness;

                    //                    timeTrack("G0291", true);
                    Mat jacobian_23_root_orient_e = get_full_vector_from_short_e( list_links[i_vc]->_kld_contact_jacobian.data.bottomRows(3), floor( (i_vc - NB_DOF_ARM) / 4 ) );

                    //                    timeTrack("G0292", true);

                    Vec v_cart_orient_torque_in_joint_e = jacobian_23_root_orient_e.transpose() * v_cart_orient_torque_in_cart;

                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                    //                    timeTrack("G03", true);

                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                    // Compute the torques corresponding to that force
                    int i_finger = floor( (i_vc - NB_DOF_ARM) / 4 );

                    Mat jacobian_23_root_e = get_full_vector_from_short_e(list_links[i_vc]->_kld_contact_jacobian.data.topRows(3), i_finger);

                    //                    timeTrack("G031", true);

                    Vec virtual_contact_torque_root_e = jacobian_23_root_e.transpose() * M2E_v(v_cart_force_pid);

                    //                    timeTrack("G03a", true);

                    v_virtual_contact_torque_new += MathLib::Vector( virtual_contact_torque_root_e.data(), virtual_contact_torque_root_e.rows() );

                    // Compute the 1-row jacobian for null space purposes.
                    Mat jacobian_1row_e = v3_proj_dir.transpose() * jacobian_23_root_e;

                    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                    //                    timeTrack("G03b", true);

                    // <<<<<<<<<<<<<<<<<<<<<<<

                    // ADD ORIENTATION STIFFNESS
                    if (b_use_ravins_method_for_vc && b_ravin_use_orientation_for_vc) virtual_contact_torque_root_e += v_cart_orient_torque_in_joint_e;

                    //          ossDebug << "virtual_contact_torque_root_e: " << virtual_contact_torque_root_e << endl;

                    // Send tasks to tasklist
                    tasklist.addTask( EgalityTask(jacobian_1row_e,
                                                  1,
                                                  virtual_contact_torque_root_e,
                                                  "Virtual contact on link " + std::to_string(i_vc),
                                                  d_task_weight_vc) );
                    ope_space_controller.addObjective( virtual_contact_torque_root_e, d_task_weight_vc, "Virtual contact on link " + std::to_string(i_vc) );

                    ope_space_controller_ho.addObjective( virtual_contact_torque_root_e.bottomRows(NB_DOF_HAND), d_task_weight_vc, "Virtual contact on link " + std::to_string(i_vc) );
                    //                    ROS_INFO_STREAM("virtual_contact_torque_root_e:(only 16 bottom are sent) \n" << virtual_contact_torque_root_e);
                    //                    timeTrack("G03c", true);

                    tasklist_hand_only.addTask( EgalityTask(jacobian_1row_e.rightCols(NB_DOF_HAND),
                                                            1,
                                                            virtual_contact_torque_root_e.bottomRows(NB_DOF_HAND),
                                                            "Virtual contact on link " + std::to_string(i_vc),
                                                            d_task_weight_vc) );

                    //                    timeTrack("G04", true);

                    // >>>>>>>>>>>>>>>>>>>>>>>
                } else {
                    cerr << "No closest contact was found, there will be a pb" << endl;

                    // That means it's only 1 contact which is the desired contact. How could it be a virtual contact then ?
                    ROS_FATAL("No closest contact found");

                    exit(0);
                }
            }
        }

    v_virtual_contact_torques = v_virtual_contact_torque_new;

    //  v_virtual_contact_torques.Print("v_virtual_contact_torques");


    // / TODO: Try this: If it goes above that value (here 2.0 N), rescale the whole vector.
    // / THIS is useless, since these variables are not used, they are just here for historical reasons
    //  double maxJointTorque = 2.0;

    //  if (v_virtual_contact_torques.Max() > maxJointTorque) {
    //    v_virtual_contact_torques = v_virtual_contact_torques / v_virtual_contact_torques.Max() * maxJointTorque;
    //    cerr << "rescaling v_virtual_contact_torques" << endl;
    //  }


    //  ossG << "]" << endl;

    //  ossGv << "]" << endl;

    timeTrack("G", b_debug);


    /**  =================================================
    * =         COMPUTE TORQUES FOR SPACE MOUSE       =
    * ================================================= */

    /**  =================================
    * =    and Get SpaceMouse values      =
    * ================================= */

    MathLib::Vector3 spacem_lin(this->twist_msg_.linear.x, this->twist_msg_.linear.y, this->twist_msg_.linear.z);
    MathLib::Vector3 spacem_rot(this->twist_msg_.angular.x, this->twist_msg_.angular.y, this->twist_msg_.angular.z);


    double d_sp_lin_gain = 0.02;
    double d_sp_rot_gain = 4.0;

    //  double d_sp_lin_gain = 0.05;
    //  double d_sp_rot_gain = 10.0;

    MathLib::Vector3 lin_vel = spacem_lin * d_sp_lin_gain;
    MathLib::Vector3 rot_vel = spacem_rot * d_sp_rot_gain;

    // Compute a velocity to be integrated
    Vector v_spacem_vel;
    v_spacem_vel.Resize(6);
    v_spacem_vel.Zero();
    v_spacem_vel.SetSubVector(0, lin_vel);
    v_spacem_vel.SetSubVector(3, rot_vel);

    // Drive the motion of the cart tracker impedance
    Eigen::Map<Eigen::Vector3d> v_spacem_linvel(lin_vel.Array(), 3);
    v_cart_pos_tracked += v_spacem_linvel * 50.0 * dt;


    Matrix3d v_des_orient_dot;

    // Method 1 (Get rotation matrix)
    v_des_orient_dot = AngleAxisd( rot_vel(0) * dt, Vector3d::UnitX() )
            * AngleAxisd( rot_vel(1) * dt,  Vector3d::UnitY() )
            * AngleAxisd( rot_vel(2) * dt, Vector3d::UnitZ() ); // This is made to take degrees ?


    /*
     * /// Method 2
     * Eigen::Matrix3d S, R_dot,R;
     * R=v_cart_orient_tracked;
     * double wx,wy,wz;
     * wx=torque(0);
     * wy=torque(1);
     * wz=torque(2);
     * S <<    0.0, -wz, wy,
     * wz,  0.0, -wx,
     * -wy, wx,  0.0;
     * R_dot=S*R;
     * e_tracked_cart_orient+=R_dot*dt;
     */

    v_cart_orient_tracked = v_des_orient_dot * v_cart_orient_tracked;


    // Compute directly or force/torque to be projected on the jacobian transpose
    Vec v_spacem_force  = M2E_v(spacem_lin) * 0.1;
    Vec v_spacem_torque = M2E_v(spacem_rot) * 0.01; // decrease

    Vec v_spacem_ft(6);
    v_spacem_ft << v_spacem_force, v_spacem_torque;
    v_spacem_ft *= 100.0;

    timeTrack("G1", b_debug);

    /**   ==============================================
     * =    Compute torques for point tracking      =
     * ============================================== */


    //  Inputs:
    //  - joint position

    //  Needs:
    //  KDL chain for the end effector kdlchain_endeff


    // / ====== COMPUTE current cart positions
    // / (v_cart_endeff_orient, v_cart_endeff_pos, ) = f(joints, kdlchain_endeff)
    // / kdlf_hand_base_pose = f(joints, kdlfksolver_hand_base)


    // Get current cartesian position/orientation
    KDL::JntArray kdlja_joint_vector          = KDL::JntArray(NB_DOF_ARM);
    KDL::JntArray kdlja_joint_vector_filtered = KDL::JntArray(NB_DOF_ARM);

    for (int j = 0; j < NB_DOF_ARM; j++) { // fill in arm positions until end of chain
        kdlja_joint_vector(j)          = v_curr_position[j];
        kdlja_joint_vector_filtered(j) = curr_position_filtered_CDD[j];
    }

    // Prepare FK solver
    KDL::ChainFkSolverPos_recursive kdlfksolver_endeff = KDL::ChainFkSolverPos_recursive(kdlchain_endeff_7_link);

    // Compute the Cartesian Position
    kdlfksolver_endeff.JntToCart(kdlja_joint_vector, kdlf_endeff_pos_7_link);

    // At the same time, get the hand base's position ...
    kdlfksolver_hand_base->JntToCart(kdlja_joint_vector, kdlf_hand_base_pose);


    // Transfer the endeffector cartesian variables in global variables.
    v_cart_endeff_orient = Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> >(kdlf_endeff_pos_7_link.M.data, 3, 3);
    v_cart_endeff_pos    = Eigen::Map<Eigen::Matrix<double, 3, 1> >(kdlf_endeff_pos_7_link.p.data, 3);

    //  ossGv << "cart position of index finger:\n " << Eigen::Map<Eigen::Matrix<double, 3, 1> >(kdlf_hand_debug.p.data, 3) << endl;


    // /////////////////////


    // / >>>> A
    // /  Impedance control
    // /
    // /
    // / ==== COMPUTE STIFFNESS MATRICES
    // Set stiffness along a certain direction.
    Eigen::Matrix3d stiffness_matrix_local, stiffness_matrix_global, stiffness_matrix_o_local, stiffness_matrix_o_global, stiffness_rotation_matrix;

    // Define normal stiffness matrices (use same gains as isotropic ...)
    stiffness_matrix_local.setIdentity();
    stiffness_matrix_local *= d_impedance_lin_stiff;
    stiffness_matrix_o_local.setIdentity();
    stiffness_matrix_o_local *= d_impedance_rot_stiff;

    // Change stiffnesses along certain directions
    //  if (b_auto_pos_tracking) {
    //    if (i_nb_allowed_contacts_status == 0) {
    //      stiffness_matrix_local(0, 0) *= 1.0; // some stiffness in x direction if no contacts
    //    } else {
    //      stiffness_matrix_local(0, 0) *= 0.0; // 0 stiffness in x direction
    //    }
    //  } else {
    //    stiffness_matrix_local(0, 0) *= 0.1;
    //  }

    if (b_auto_pos_tracking) {
        // dont care for now
    } else {
        if (i_nb_allowed_contacts_status == 0) {
            //      ossG << "normal mode" << endl;
            // if not contact, don't touch it
        } else {
            //      ossG << "low impedance mode" << endl;

            // decrease all position values .
            //            stiffness_matrix_local(0,0) *=0.05;
            //            stiffness_matrix_local(1,1) *=0.05;
            //            stiffness_matrix_local(2,2) *=0.05;


            // if there is a contact, jsut vary along contact direction: not stiffness or little ...
            //      stiffness_matrix_o_local(0, 0) *= 0.05; // very low stiffness in x orientation
            //      stiffness_matrix_o_local(1, 1) *= 0.05; // lower stiffness in y orientation // test ..
            stiffness_matrix_o_local(0, 0) *= 0.5; // very low stiffness in x orientation
            stiffness_matrix_o_local(1, 1) *= 0.5; // lower stiffness in y orientation // test ..
        }
    }


    // / UNTIL HERE CAN BE PRECOMPUTED .... >>>>

    // / ====== impedance_tracking_torques = f(v_cart_pos_tracked_filtered, kdlja_joint_vector_filtered, myImpedanceController)
    timeTrack("G1'2", b_debug);

    // Express the stiffness matrix in the endeffector frame, or the normal frame ?
    if (1) stiffness_rotation_matrix = v_cart_endeff_orient;
    else stiffness_rotation_matrix = normal_Frame;


    // For debug, and tests.
    bool stiffness_in_world = true;

    if (stiffness_in_world) {
        stiffness_rotation_matrix.setIdentity();
    }

    // Transform stiffness matrices to global frame
    stiffness_matrix_global   = stiffness_rotation_matrix * stiffness_matrix_local * stiffness_rotation_matrix.transpose();
    stiffness_matrix_o_global = stiffness_rotation_matrix * stiffness_matrix_o_local * stiffness_rotation_matrix.transpose();


    // cerr << "stiffness_matrix_global: \n" << stiffness_matrix_global << endl;
    // cerr << "stiffness_matrix_o_global: \n" << stiffness_matrix_o_global << endl;

    timeTrack("G1'3", b_debug);

    if (!b_use_isotropic_impedance) {
        myImpedanceController->SetStiffnessPosition_e(stiffness_matrix_global);
        myImpedanceController->SetStiffnessOrientation_e(stiffness_matrix_o_global);
    }


    // change the switch for this one ... still not very good ....
    if (!b_use_point_tracking) {
        myImpedanceController->SetTarget_e(v_cart_pos_tracked_ravin, v_cart_orient_tracked_ravin);

        //  if (b_ravins_mode) myImpedanceController->SetTarget_e(v_cart_pos_tracked_ravin, v_cart_orient_tracked_ravin);
    } else {
        if (!b_use_filtered_cartesian_target) {
            myImpedanceController->SetTarget_e(v_cart_pos_tracked, v_cart_orient_tracked);
        } else {
            // Orientation filtering is not good still.
            myImpedanceController->SetTarget_e(v_cart_pos_tracked_filtered, v_cart_orient_tracked_filtered);

            //      myImpedanceController->SetTarget_e(v_cart_pos_tracked_filtered, v_cart_orient_tracked);
        }
    }


    //  myImpedanceController->Update_e(kdlja_joint_vector); // This line takes time because the solver is not optimized ...
    myImpedanceController->Update_e(kdlja_joint_vector_filtered); // This line takes time because the solver is not optimized ...
    Vec impedance_tracking_torques;
    myImpedanceController->GetOutput_e(impedance_tracking_torques);

    timeTrack("G1a", b_debug);


    // /
    // / Second method: use directly values from the spacemouse
    // /
    // Compute the Jacobian
    KDL::ChainJntToJacSolver kdljacsolver_endeff = KDL::ChainJntToJacSolver(kdlchain_endeff_7_link);

    KDL::Jacobian kdljac_endenff;
    kdljac_endenff.resize( kdlchain_endeff_7_link.getNrOfJoints() );
    kdljacsolver_endeff.JntToJac(kdlja_joint_vector, kdljac_endenff);

    Vec spacemouse_endeff_torques = kdljac_endenff.data.transpose() * v_spacem_ft;

    timeTrack("G1b", b_debug);

    // Either get the point tracking, or directly the projected values from spacemouse ...
    // Add the torques to the Priority controller
    Vec tracking_torques;

    if (b_use_point_tracking) {
        tracking_torques = impedance_tracking_torques;
    } else {
        tracking_torques = spacemouse_endeff_torques;
    }

    Vec tracking_torques_23 = Vec::Zero(NB_DOF_TOT);
    tracking_torques_23.head( tracking_torques.rows() ) = tracking_torques;

    Vec tracking_jacobian = Vec::Zero(NB_DOF_TOT);
    tracking_jacobian.head(NB_DOF_ARM) = Vec::Ones(NB_DOF_ARM);

    //                                tasklist.addTask( EgalityTask(Mat::Zero(1, NB_DOF_TOT),
    tasklist.addTask( EgalityTask(tracking_jacobian.transpose(),
                                  1,
                                  tracking_torques_23,
                                  "space-mouse / point tracking torques", d_task_weight_space_mouse) );

    // TODO Debug
    if(!b_follow_planner_joint_position)
        ope_space_controller.addObjective(tracking_torques_23, d_task_weight_space_mouse, "space-mouse / point tracking torques");


    Vector v_spacem_torques;
    v_spacem_torques.Resize(NB_DOF_ARM);
    v_spacem_torques.Zero();
    v_spacem_torques = MathLib::Vector( tracking_torques_23.data(), tracking_torques_23.rows() );

    // DEBUG: when using short hand control ..
    if (b_handonly_nullspace && !b_ravin_jointpos_reach) {
        v_additionnal_torques += v_spacem_torques;
    }

    timeTrack("g2", b_debug);


    /** ====================================================
     * =   Compute desired joint position in pd exploration  =
     * ==================================================== */

    v_explore_joint_pos_simple; // Compute this


    // initial position is v_ravin_joint_pos_tracked
    // maybe do it in velocity: it will be easier ...
    double d_joint_velocity = d_joint_close_velocity * M_PI / 180.0; // 15deg/sec


    //  if(!b_close_allowed){
    // not sure about this check ..
    if ( !b_close_allowed || (v_contact_on_link_allowed.Sum() > 0) ) {
        // reset the position of the fingers when not in close mode
        v_explore_joint_pos_simple = v_ravin_joint_pos_tracked;
    } else {
        for (int finger = 0; finger < NB_FINGERS; finger++) {
            // For each finger
            bool b_finger_contact = true;
            int  offset_finger    = finger * NB_DOF_FINGER + NB_DOF_ARM;

            if (v_finger_contact_max_dof[finger] == -1) {
                b_finger_contact = false;
            }

            // on the thumb, a contact on the base finger is not considered a contact (the following links should still close)
            if ( (finger == 3) && (v_finger_contact_max_dof[finger] < 2) ) {
                b_finger_contact = false;
            }

            //      ossDebug << "finger: " << finger << endl;
            //      ossDebug << "b_finger_contact: " << b_finger_contact << endl;
            //      ossDebug << "v_finger_contact_max_dof[finger]: " << v_finger_contact_max_dof[finger] << endl;

            if (i_pd_grasp == 1) {
                // Close joints 1 by one

                // Find which joint to move: (depends on the finger, does not work for the thumb ...)
                int i_joint_to_move;

                if (!b_finger_contact) {
                    // if there is no contact, first valid one
                    if (finger == 3) i_joint_to_move = 2;
                    else i_joint_to_move = 1;
                } else {
                    // if there is a contact, the one after the last ()
                    i_joint_to_move = v_finger_contact_max_dof[finger] + 1;
                }

                if (i_joint_to_move < 4) {
                    //          ossDebug << "Moving this link ...!" << endl;

                    // Increase position of that joint
                    v_explore_joint_pos_simple[offset_finger + i_joint_to_move] += d_joint_velocity * dt;
                }
            } else if (i_pd_grasp == 2) {
                // Close all joints together


                //        ossDebug << "\n ---- \nFinger:  " << finger << endl;
                //        ossDebug << "v_finger_contact_max_dof[finger] " << v_finger_contact_max_dof[finger] << endl;
                for (int i_finger_joint = 1; i_finger_joint < 4; i_finger_joint++) {
                    if ( (finger != 3) || (i_finger_joint > 1) ) {
                        // if thumb, joint should be higher
                        //            ossDebug << "joint " << i_finger_joint << endl;
                        if (v_finger_contact_max_dof[finger] < i_finger_joint) {
                            //              ossDebug << "increase joint value ... " << endl;
                            // If there is no contact above this link, increase its position
                            v_explore_joint_pos_simple[offset_finger + i_finger_joint] += d_joint_velocity * dt;
                        }
                    }
                }
            }
        }
    }

    /** ====================================================
     * =   OBTAIN DESIRED POSITION FOR POSITION CONTROL  =
     * ==================================================== */


    Vec v_hand_desired_pos(NB_DOF_HAND);
    v_hand_desired_pos.setZero();

    Vec v_arm_desired_pos(NB_DOF_ARM);
    v_arm_desired_pos.setZero();

    /// This is bad, should be changed for something much simpler.
    if ( b_ravin_jointpos_reach || b_ravin_close_mode || ( b_ravin_cart_reach && (i_pd_grasp == 0) ) ) {
        // get value defined by Ravin during reaching
        v_hand_desired_pos = v_ravin_joint_pos_tracked.tail(NB_DOF_HAND);
        v_arm_desired_pos  = v_ravin_joint_pos_tracked.head(NB_DOF_ARM);
        //        ROS_INFO_STREAM("Copy v_ravin_joint_pos_tracked ");
        //        ossGv << "copy v_ravin_joint_pos_tracked: " << 0 << endl;

    } else if (b_ravin_grasped_mode || b_ravin_task_mode) {
        // During grasping and task get the grasping pose
        v_hand_desired_pos = v_ravin_joint_pos_grasped.tail(NB_DOF_HAND);
        v_arm_desired_pos  = v_ravin_joint_pos_grasped.head(NB_DOF_ARM);
    } else if (i_pd_grasp > 0) {
        // During grasping and task get the grasping pose
        v_hand_desired_pos = v_explore_joint_pos_simple.tail(NB_DOF_HAND);
        v_arm_desired_pos  = v_ravin_joint_pos_tracked.head(NB_DOF_ARM); // does not change
    } else {
        // get v_hand_close_posture: open position of the hand ...
        v_hand_desired_pos = v_hand_close_posture * PI / 180.0;
        v_arm_desired_pos  = M2E_v( v_curr_position.GetSubVector(0, NB_DOF_ARM) ); // Set current position (seems a bit wrong..., should not be updated ...)
        // TODO test
        //        v_arm_desired_pos  = desired_planned_joint_position; // Set current position (seems a bit wrong..., should not be updated ...)
    }

    //    cerr << "v_hand_close_posture: \n " << v_hand_close_posture << endl;
    //    cerr << "v_hand_desired_pos: \n " << v_hand_desired_pos << endl;

    // TODO: this won't work if I use anything else.
    // If I start receiving desired hand position from outside, it takes the override ...
    if(b_got_hand_explo_posture){
        v_hand_desired_pos = v_hand_explo_posture;
        //        ROS_INFO_STREAM_THROTTLE(1.0,"Copy v_hand_explo_posture ");
    }

//    ROS_INFO_STREAM("v_hand_desired_pos: " << v_hand_desired_pos);

    timeTrack("g2a", b_debug);
    // / -----------------------------------------
    // TODO DEBUG: set desired joint pos to 0
    //    v_arm_desired_pos.setZero();
    //    v_arm_desired_pos = desired_planned_joint_position;


    // / NEW
    // Get the current position of the arm's desired position from the filter.
    cdd_desired_joint_pos_arm->SetTarget( E2M_v(v_arm_desired_pos) );
    cdd_desired_joint_pos_arm->Update();
    cdd_desired_joint_pos_arm->GetState(v_desired_arm_pos_filtered);

    cdd_desired_joint_pos_hand->SetTarget( E2M_v(v_hand_desired_pos) );
    cdd_desired_joint_pos_hand->Update();
    cdd_desired_joint_pos_hand->GetState(v_desired_hand_pos_filtered);


    cdd_cartesian_position_motion->SetTarget( E2M_v(v_cart_pos_tracked) );
    cdd_cartesian_position_motion->Update();

    cdd_cartesian_orientation_motion->SetTarget( E2M_v( v_cart_orient_tracked.eulerAngles(2, 1, 2) ) );
    cdd_cartesian_orientation_motion->Update();


    // Not sure that this works very well ...
    // ( I had discontinuities in the robot motion)
    // not sure where the error is
    // disabled for now
    MathLib::Vector vec(4);
    vec(0) = Eigen::Quaterniond(v_cart_orient_tracked).w();
    vec(1) = Eigen::Quaterniond(v_cart_orient_tracked).x();
    vec(2) = Eigen::Quaterniond(v_cart_orient_tracked).y();
    vec(3) = Eigen::Quaterniond(v_cart_orient_tracked).z();
    cdd_cartesian_orientationQ_motion->SetTarget(vec);
    cdd_cartesian_orientationQ_motion->Update();


    MathLib::Vector temp_vec;
    cdd_cartesian_position_motion->GetState(temp_vec);
    v_cart_pos_tracked_filtered = M2E_v(temp_vec);

    //  MathLib::Vector temp_vec2;
    //  cdd_cartesian_orientation_motion->GetState(temp_vec2);
    //  // Using Z-Y-Z (2-1-2)
    //  v_cart_orient_tracked_filtered = AngleAxisd( temp_vec2(0), Vector3d::UnitZ() )
    //                                   * AngleAxisd( temp_vec2(1),  Vector3d::UnitY() )
    //                                   * AngleAxisd( temp_vec2(2), Vector3d::UnitZ() ); // This is made to take degrees ?

    //
    MathLib::Vector temp_vec3;
    cdd_cartesian_orientationQ_motion->GetState(temp_vec3);
    Quaterniond quat( temp_vec3(0), temp_vec3(1), temp_vec3(2), temp_vec3(3) );
    quat.normalize();

    // Need to normalize the orientation first
    v_cart_orient_tracked_filtered = quat.toRotationMatrix();


    if (v_cart_pos_tracked_filtered.norm() > 100) {
        cerr << " if (v_cart_pos_tracked_filtered.norm() > 100)" << endl;
        ROS_FATAL("(v_cart_pos_tracked_filtered.norm() > 100)");

        exit(0);
    }

    // TODO test: Ignore the filter
    v_desired_arm_pos_filtered = E2M_v(v_arm_desired_pos);
    // / RECONSTRUCT ..
    v_desired_pos_rad_Filtered.SetSubVector(0,          v_desired_arm_pos_filtered);
    v_desired_pos_rad_Filtered.SetSubVector(NB_DOF_ARM, v_desired_hand_pos_filtered);

    timeTrack("H", b_debug);


    /**  =================================
    * =        POSITION CONTROL       =
    * ================================= */
//v_desired_pos_rad_Filtered.Print("v_desired_pos_rad_Filtered");
//curr_position_filtered_CDD.Print("curr_position_filtered_CDD");
//ROS_INFO_STREAM("v_desired_pos_rad_Filtered: " <<    );

    // Filtered version:
    pid_controller_joint.SetTarget(v_desired_pos_rad_Filtered);
    pid_controller_joint.SetInput(curr_position_filtered_CDD); // TODO: try with less filtered version ?

    pid_controller_joint.Update(dt);
    pid_controller_joint.GetOutput(v_torques_for_joint_error); // should only be for joints where it is deired (some PID ...)

//v_torques_for_joint_error.Print("v_torques_for_joint_error");

    // Add those torques to the nullspace controller
    if (b_finger_pd_in_nullspace) {
//        double d_ns_torques_multiplier = 0.3; // PD torques are decreased.
        Vec    torque_pid_fingers              = M2E_v(v_torques_for_joint_error) * d_ns_torques_multiplier;

        // put torques on arm to 0 ?
        torque_pid_fingers.head(NB_DOF_ARM) = VectorXd::Zero(NB_DOF_ARM);

        // Create weight vector
        Vec weights_finger = VectorXd::Constant(NB_DOF_TOT, d_task_weight_joint_position);
        weights_finger.head(NB_DOF_ARM) = VectorXd::Constant(NB_DOF_ARM, 0.0001); // small value on the arm (don't care for their result..)
        ope_space_controller.addObjective(torque_pid_fingers, weights_finger, "PID torques finger");
        //        ope_space_controller_ho.addObjective(torque_pid_fingers.bottomRows(NB_DOF_HAND), weights_finger.bottomRows(NB_DOF_HAND), "PID torques finger");
        ope_space_controller_ho.addObjective(torque_pid_fingers.bottomRows(NB_DOF_HAND), d_task_weight_joint_position, "PID torques finger");
        // try to replace it.
        //                ROS_INFO_STREAM("torque_pid_fingers.bottomRows(NB_DOF_HAND): " << torque_pid_fingers.bottomRows(NB_DOF_HAND).transpose());
        // This measure is fine


//        ROS_INFO_STREAM("torque_pid_fingers: " << torque_pid_fingers);

//        v_desired_pos_rad_Filtered.Print("v_desired_pos_rad_Filtered");
    }

    // Add objective to follow arm PD controller. TODO: finish this, needs a modification to add shorter objectives
    // This is for when using the planner. So not used anymore
    if(b_follow_planner_joint_position){
        double d_ns_torques_multiplier = 1.0;
        Vec    torque_pid_arm          = M2E_v(v_torques_for_joint_error) * d_ns_torques_multiplier;

        // put torques on finger to 0 ?
        torque_pid_arm.tail(NB_DOF_HAND) = VectorXd::Zero(NB_DOF_HAND);

        // Create weight vector
        Vec weights_arm = VectorXd::Constant(NB_DOF_TOT, d_task_weight_joint_position);
        weights_arm.tail(NB_DOF_HAND) = VectorXd::Constant(NB_DOF_HAND, 0.0001); // small value on the fingers

        //        ope_space_controller.addObjective(torque_pid_arm, weights_arm, "PID torques arm");

        v_additionnal_torques += E2M_v(torque_pid_arm);

    }


    // Zero torques for non PD-controller joints:
    for (int i = 0; i < NB_DOF_TOT; i++) {
        if ( (jointCtrlMode[i] == POSITION_PID) ) {}
        else {
            v_torques_for_joint_error[i] = 0;
        }
    }

    //    v_torques_for_joint_error.Print("v_torques_for_joint_error");


    // Save last iteration info
    v_prev_position                 = v_curr_position;
    v_prev_velocity                 = v_curr_velocity;
    v_des_diff_rad_Filtered         = (v_desired_pos_rad_Filtered - v_prev_desired_pos_rad_Filtered); // It gives the direction of desired position
    v_prev_desired_pos_rad_Filtered = v_desired_pos_rad_Filtered;

    for (int i = 0; i < NB_DOF_TOT; i++) {
        prevJointCtrlMode[i] = jointCtrlMode[i];
    }


    timeTrack("I", b_debug);


    /**  ===========================================
    * =    NULL SPACE TORQUES COMPUTATION       =
    * =========================================== */


    // / Compute total torques to project
    //    Vector v_total_torques_to_project; // / Todo: global?
    //    v_total_torques_to_project.Resize(NB_DOF_TOT);
    //    v_total_torques_to_project.Zero();
    //    v_total_torques_to_project.SetSubVector(0, v_spacem_torques);

    //    if (b_use_virtual_contact_torques && !b_only_gravity_compensation) {
    //        v_total_torques_to_project += v_virtual_contact_torques;
    //    }


    // New nullspace: signal a go (all data is ready (actually do it earlier ?)).
    {
        // Protect the copy of the tasklist
        std::lock_guard<std::mutex> guard2(bm_mutex_main_c);

        if (i_counter > 10) {
            // Since I apply a lock here, copy the task list to the one we will use
            // Choose between full task list, and short one (only the hand)
            if (b_handonly_nullspace){
                tasklist_permanent = tasklist_hand_only;
                ope_space_controller_permanent = ope_space_controller_ho;
                //                                ROS_INFO_STREAM("handONLY");
            }else{
                tasklist_permanent = tasklist;
                ope_space_controller_permanent = ope_space_controller;
                //                ROS_INFO_STREAM("handONLY NOT (wrong one)");
            }
            //            ope_space_controller_permanent = ope_space_controller;

            // Pass back the no-projection parameter to the copy
            tasklist_permanent.b_no_projection = b_no_projection;

            b_new_Jacobian_c = true;
            bc_condition_c.notify_all();
        }
    }


    Vec tot_torques(NB_DOF_TOT);
    tot_torques.setZero();

    // If using ravin's method, should be only during closing ?
    // TODO: why this does not happen in other cases ???
    if ( (i_counter > 10) && (!b_use_ravins_method_for_vc || b_ravin_close_mode || b_close_allowed) ) {
        // Get back the torques from the other thread.
        {
            std::lock_guard<std::mutex> lock_c(bm_mutex_main_c);

            // Retrieve the torques
            if ( tot_torques_permanent.size() ) {
                if (b_handonly_nullspace){
                    tot_torques.tail(NB_DOF_HAND) = tot_torques_permanent;
                }
                else{
                    tot_torques = tot_torques_permanent;
                }
            }

            //                        ossGv << "tot_torques just after returning the permanent: \n" << tot_torques << endl;
        }
    }
//    ROS_INFO_STREAM("tot_torques: " << tot_torques);


    // / Forces for finger PID position
    if (!b_finger_pd_in_nullspace) {
        v_additionnal_torques += v_torques_for_joint_error;
    }

    //    v_torques_for_joint_error.Print("v_torques_for_joint_error");

    //        ossGv << "tot_torques before being added to addition torques: \n " << tot_torques << endl;


    // Remove thumb projected torques (only stay in place) (for exploration)
    //    bool b_no_thumb_from_opt = true;
    if(b_no_thumb_from_opt){
        tot_torques.tail(NB_DOF_FINGER).setZero();
    }


    v_additionnal_torques += MathLib::Vector( tot_torques.data(), tot_torques.rows() );



    // Remove Coriolis forces (probably already done on kuka)
    // THIS IS BUGGY , DO NOT USE, TODO: DEBUG
    if (b_compensate_coriolis) {
        v_additionnal_torques += v_torques_coriolis; // not sure: + or - ?
    }


    timeTrack("J", b_debug);


    /**  ===========================================
    * =  ADD FEEDFORWARD TORQUES FOR FRICTION   =
    * =========================================== */

    // / Solution 1: add constant torques
    //    double frictionTorque;
    //    frictionTorque=0.01; /* this is a very random value */
    // //    frictionTorque=0.02; /* this is a very random value  */ // But it has a huge effect
    // //    frictionTorque=0.01; /* this is a very random value  */ // But it has a huge effect
    //    for (int i=0;i<NB_DOF_TOT;i++){ // maybe not for kuka ? ok why not
    //        if(v_additionnal_torque[i]>0)
    //            v_additionnal_torque[i]+=frictionTorque;
    //        else if (v_additionnal_torque[i]<0)
    //            v_additionnal_torque[i]-=frictionTorque;
    //    }


    // / Solution 2: add (proportionnal)? torques

    /* Some feedworward Torque computation */

    //    mDesiredDifferenceRadFiltered.Print("mDesiredDifferenceRadFiltered");
    double d_friction_fw = 0.03;

    for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) { // only for fingers
        if (v_des_diff_rad_Filtered[i] > 0.0001) {
            v_additionnal_torque_friction[i] += d_friction_fw;
        } else if (v_des_diff_rad_Filtered[i] < -0.0001) {
            v_additionnal_torque_friction[i] -= d_friction_fw;
        }
    }

    //    v_additionnal_torque+=v_additionnal_torque_friction;
    //    v_additionnal_torque_friction.Print("v_additionnal_torque_friction");


    /* Safety: threshold the torques to send  */

    timeTrack("K", b_debug);

    v_gravComp_torque *= d_grav_comp_multiplier;


    /**  ==================================
    * =  SEND TORQUES TO ALLEGROHAND   =
    * ================================== */

    //  v_gravComp_torque.Print("grav comp");
    //    ossGv << "v_additionnal_torques: " << M2E_v(v_additionnal_torques) << endl;

    if (i_counter > 100) {                 // leave some time for initializations
        if (b_only_gravity_compensation) { // TODO: this is maybe not working for gazebo simulation
            v_total_torque = v_gravComp_torque;
            b_was_disabled = 1;
        } else {
            v_total_torque = v_gravComp_torque + v_additionnal_torques;

            if (b_was_disabled) {
                b_was_disabled = 0;
            }
        }
    }


    v_total_torque_a = v_total_torque.GetSubVector(NB_DOF_ARM, NB_DOF_HAND);


    // ////////////////////

    /*
     * if (b_use_allegro) canDevice->setTorque( v_total_torque_a.Array() );
     *
     * if (b_use_allegro)
     * if ( b_allegro_success && (i_no_torque == 0) ) {
     * canDevice->_writeDevices();
     * }
     */

    // //////////////

    /**  ==================================
     * =  SEND CONTROLS TO THE ROBOT    =
     * ==================================
     *
     * ///     (ONLY IF NOT IN SIMULATION ....)
     *
     * /// Need: a choice of Control Mode (but depending on the program, not the robot:)
     * /// a) Everything in Torque control
     * ///     Make sure to have securities on torques and velocities, in addition to the ones from KUKA
     *
     * /// b) Arm in X-mode, AllegroHand in torque mode
     * ///     Choose wether to put arm's columns to 0 --> assume it is in position control ---> TODO: ADD A SWITCH
     * ///     Arm could be in:
     * ///         Gravity Compensation
     * ///         Position Control (keeping a predefined target position)
     * ///         Position Control (following a target position)
     * ///         Impedance Control (same ...)
     *
     */
    timeTrack("L", b_debug);


    //  Vector v_short_torques(NB_DOF_ARM, false);
    //  v_additionnal_torques.GetSubVector(0, NB_DOF_ARM, v_short_torques);

    //  ossGv << "i_counter: " << i_counter << endl;
    //  ossGv << "ctrlm_robot: " << ctrlm_robot << endl;
    //  ossGv << "ctrlm_robot_prev: " << ctrlm_robot_prev << endl;
    //  ossGv << "ctrlm_robot_next: " << ctrlm_robot_next << endl;

    //  cout << "i_counter: " << i_counter << endl;
    //  cout << "ctrlm_robot: " << ctrlm_robot << endl;
    //  cout << "ctrlm_robot_prev: " << ctrlm_robot_prev << endl;
    //  cout << "ctrlm_robot_next: " << ctrlm_robot_next << endl;

    //  if(i_counter==15){
    //    exit(0);
    //  }

    //  if(!b_use_gazebo)
    if (b_control_real_robot)
        switch (ctrlm_robot) {
        {
        case POSITION_PID_STATIC:

                if (ctrlm_robot_prev != POSITION_PID_STATIC) { // If newly in this mode, save the current position
                    v_last_position_static = v_curr_position;
                    cerr << "Registering a new static position" << endl;
                    ROS_WARN_STREAM("Setting new static position");
                }

                if (mRobot->GetControlMode() != Robot::CTRLMODE_JOINTIMPEDANCE) {
                    mRobot->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);
                    ROS_WARN_STREAM("Switching to CTRLMODE_JOINTIMPEDANCE control mode (from POSITION_PID_STATIC)");
                }

                mActuatorsGroup.SetJointAngles(v_last_position_static);
                break;
        }

        {
        case POSITION_PID: // never used for now

                if (mRobot->GetControlMode() != Robot::CTRLMODE_JOINTIMPEDANCE) {
                    mRobot->SetControlMode(Robot::CTRLMODE_JOINTIMPEDANCE);
                    ROS_WARN_STREAM("Switching to CTRLMODE_JOINTIMPEDANCE control mode (from POSITION_PID)");
                }

                mActuatorsGroup.SetJointAngles(v_desired_pos_rad_Filtered); // test that ...

                break;
        }


        {
        case TORQUE: // with joint imepdance

                if (mRobot->GetControlMode() != Robot::CTRLMODE_TORQUE) {
                    mRobot->SetControlMode(Robot::CTRLMODE_TORQUE);
                    ROS_WARN_STREAM("Switching to CTRLMODE_TORQUE control mode (from TORQUE)");
                }

                // Copy and Send torques
                Vector v_short_torques(NB_DOF_ARM, false);

                // In case the torque mode is only use for impedance control.
                //                bool b_only_impedance = true;
                //                bool b_only_impedance = false;
                bool b_only_impedance = b_handonly_nullspace;

                if (b_only_impedance) {
                    v_short_torques = E2M_v(impedance_tracking_torques);
                } else {
                    v_additionnal_torques.GetSubVector(0, NB_DOF_ARM, v_short_torques); // Maybe I should exclude the friction in this
                }


                ( (LWRRobot *)mRobot )->SetCommandedJointTorques(v_short_torques);

                break;
        }

        {
        case GRAV_COMP:

                if (mRobot->GetControlMode() != Robot::CTRLMODE_GRAVITYCOMPENSATION) {
                    mRobot->SetControlMode(Robot::CTRLMODE_GRAVITYCOMPENSATION);
                    ROS_WARN_STREAM("Switching to CTRLMODE_GRAVITYCOMPENSATION control mode (from GRAV_COMP)");
                }

                break;
        }

        {
        case CART_IMP:

                if (mRobot->GetControlMode() != Robot::CTRLMODE_CARTIMPEDANCE) {
                    mRobot->SetControlMode(Robot::CTRLMODE_CARTIMPEDANCE);
                    ROS_WARN_STREAM("Switching to CTRLMODE_CARTIMPEDANCE control mode (from GRAV_COMP)");

                    MathLib::Vector3 lwr_cart_position;
                    MathLib::Matrix3 lwr_cart_orientation;
                    ( (LWRRobot *)mRobot )->GetMeasuredCartPose(lwr_cart_position, lwr_cart_orientation);
                    ( (LWRRobot *)mRobot )->SetCartCommand(lwr_cart_position, lwr_cart_orientation);
                }
                ossGv << "Now trying to control robot in cart imp mode ..." << endl;

                // not used for now
                // only do that if the command is ready ...
                //        if (b_cart_command_ready) {
                //          ( (LWRRobot *)mRobot )->SetCartCommand( E2M_v3(lwr_des_cart_position_e), E2M_m3(lwr_des_cart_orientation_e) );
                //        }
                break;
        }
        }


    //  cout << "passed the switch loop : " << i_counter << endl;

    mActuatorsGroup.WriteActuators();

    timeTrack("M", b_debug);


    /**  =============================
    * =     Publish some data     =
    * ============================= */


    //    if(!M2E_v(v_additionnal_torques).allFinite()){
    //        ROS_FATAL_STREAM("v_additionnal_torques: " << M2E_v(v_additionnal_torques));
    //        exit(0);
    //    }

    //    if(v_)
    //    ROS_INFO_STREAM("v_additionnal_torques: " << M2E_v(v_additionnal_torques).transpose());
    //    ROS_FATAL_STREAM("v_torques_for_joint_error: " << M2E_v(v_torques_for_joint_error).transpose());
    //    ROS_FATAL_STREAM("tot_torques: " << tot_torques.transpose());
    //    if(v_additionnal_torques.Norm() > 20.0){
    //    if(v_curr_position_tempgaz.GetSubVector(0,5).Norm()<0.01){

    //        ROS_FATAL_STREAM("v_additionnal_torques: " << M2E_v(v_additionnal_torques));
    //        ROS_FATAL_STREAM("v_additionnal_torques.Norm: " << v_additionnal_torques.Norm());
    //        exit(0);
    //    }

    // / ----------------------------------
    // / == Send joint torques to Gazebo
    if (b_use_gazebo) {
        std_msgs::Float32MultiArray array;

        if (i_no_torque == 0) {                                     // Dont send torque this time
            if (i_counter > 500.0) {

                if (!b_simulate_hand_only){
                    for (int i = 0; i < NB_DOF_TOT; i++) {
                        array.data.push_back(v_additionnal_torques[i]); // pb: This contains gravity compensation torques for allegro_hand ...
                    }
                }else{
                    for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) {
                        array.data.push_back(v_additionnal_torques[i]); // pb: This contains gravity compensation torques for allegro_hand ...
//                        array.data.push_back(v_total_torque[i]); // pb: This contains gravity compensation torques for allegro_hand ...
                    }
                }

                pub_torques.publish(array);
            }
        }
    }
    timeTrack("N1: pub gaz torques", b_debug);


    // / ----------------------------------
    // / == Send torques to the real allegro Hand
    allegro_hand_state_desired_torque.header.stamp = ros::Time::now();


    //    bool b_disable_index = true;

    // set index torques to 0 ...
    if (b_disable_index){
        v_total_torque_a.SetSubVector(0,MathLib::Vector(4));
    }

    for (int i = 0; i < NB_DOF_HAND; i++) {
        allegro_hand_state_desired_torque.effort[i]   = v_total_torque_a[i];
        allegro_hand_state_desired_torque.velocity[i] = 0.0;
        allegro_hand_state_desired_torque.position[i] = 0.0;
    }




    pub_hand_state_des_torque.publish(allegro_hand_state_desired_torque);

    timeTrack("N2: pub hand des state", b_debug);


    // / ----------------------------------
    // / == Send point cloud data (for plotting in Rviz)
    if (1) {
        //        PointCloud::Ptr msg(new PointCloud);
        //        msg->header.frame_id = "/world";

        //        for (int i = 0; i < i_nb_contacts; i++) {
        //            msg->points.push_back( pcl::PointXYZ( list_contacts[i]->_kdl_frame.p.x(), list_contacts[i]->_kdl_frame.p.y(), list_contacts[i]->_kdl_frame.p.z() ) );
        //        }
        //        msg->height = 1;
        //        msg->width  = msg->points.size();

        //        // Don't publish if not points, and publish less often
        //        if ( msg->points.size() && ( (i_counter % 10) == 0 ) ) {
        //            pub_pcl.publish(msg);
        //        }

        pcl::PointCloud<pcl::PointNormal> points_xyzn;

        for (int i = 0; i < i_nb_contacts; i++) {
            pcl::PointNormal pt;
            pt.x = list_contacts[i]->_kdl_frame.p.x();
            pt.y = list_contacts[i]->_kdl_frame.p.y();
            pt.z = list_contacts[i]->_kdl_frame.p.z();

            pt.normal_x = list_contacts[i]->normalInBaseFrame()[0];
            pt.normal_y = list_contacts[i]->normalInBaseFrame()[1];
            pt.normal_z = list_contacts[i]->normalInBaseFrame()[2];

            points_xyzn.points.push_back(pt);
        }

        points_xyzn.header.frame_id = "/world";
        points_xyzn.height          = 1;
        points_xyzn.width           = points_xyzn.points.size();

        if ( points_xyzn.points.size() && ( (i_counter % 10) == 0 ) ) {
            pub_pcl.publish(points_xyzn);
        }
    }

    // / ----------------------------------
    // / == Send Cartesian target Frame to Gazebo for drawing
    if (b_use_gazebo && 0) {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = "Indicator";

        tf::pointEigenToMsg(v_cart_pos_tracked, modelState.pose.position);
        tf::quaternionEigenToMsg(Eigen::Quaterniond(v_cart_orient_tracked), modelState.pose.orientation);

        if (b_use_point_tracking) pub_cart_target_gazebo.publish(modelState);
    }


    // / --------------------------------------------------------------------
    // / == Also publish the desired position final to gazebo (DEBUG):

    if (b_use_gazebo && 0) {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = "des_pos_indicator";

        tf::pointEigenToMsg(e_final_pos_exploration, modelState.pose.position);
        tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(), modelState.pose.orientation);


        if (!b_use_ravins_method_for_vc) {
            pub_cart_target_gazebo.publish(modelState); // Reuse the same publisher and Message as last one.
        }
        timeTrack("N3: pub gaz more stuff", b_debug);
    }


    // / --------------------------------------------------------------------
    // / == Send Cartesian Frame of the Hand, or else,  to Gazebo for DEBUG (rotations ...)
    if (b_use_gazebo && 0) {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = "IndicatorDebug";

        //    tf::poseKDLToMsg(kdlf_hand_base_pose, modelState.pose);
        tf::poseKDLToMsg(kdlf_endeff_pos_7_link, modelState.pose);

        pub_cart_target_gazebo.publish(modelState);
    }

    // Do the same in Rviz ...


    // / --------------------------------------------------------------------
    // / Publish position of virtual contact frames if they exist. (for gazebo) //Deactivated: too slow

    if (0) {
        gazebo_msgs::ModelState modelState;

        for (int i = 0; i < NB_DOF_TOT; i++) {
            modelState.model_name = "ref_" + std::to_string(i);

            if ( v_contact_on_link_allowed(i) ) {
                modelState.pose.position.x = list_links[i]->_kdl_frame.p.x();
                modelState.pose.position.y = list_links[i]->_kdl_frame.p.y();
                modelState.pose.position.z = list_links[i]->_kdl_frame.p.z();
            } else {
                modelState.pose.position.x = 0.0;
                modelState.pose.position.y = 0.0;
                modelState.pose.position.z = -0.1;
            }
            pub_cart_target_gazebo.publish(modelState); // Reuse the same publisher and Message as last one.
        }
    }
    timeTrack("N4: pub cart target", b_debug);

    // / --------------------------------------------------------------------
    // /  Publish contact INFO
    if (1) {
        if ( (i_counter % 10 == 0) ) {
            //      if ( (i_counter % 50 == 0) ) {
            std_msgs::Float32MultiArray my_array_tactile_pressures;

            for (int i = 0; i < NB_DOF_TOT; i++) my_array_tactile_pressures.data.push_back(list_links[i]->d_pressure);

            for (int i = 0; i < NB_DOF_TOT; i++) my_array_tactile_pressures.data.push_back(list_links[i]->contact_status); // plot contact status too ...

            pub_tactile_pressures.publish(my_array_tactile_pressures);
        }
    }

    // /------------------------------------------------------------------------------------
    // / Publish transform from "ahand link palm" to the world
    // This is necessary because most information is expressed in that frame when displayed in Rviz.
    {
        geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform( kdlf_hand_base_pose.Inverse() );
        tf_transform.header.stamp    = ros::Time::now();
        tf_transform.header.frame_id =   tf::resolve("", "kdlf_hand_base_pose");
        tf_transform.child_frame_id  = tf::resolve("", "world");
        if(0) // dont send (DEBUG)
            br->sendTransform(tf_transform);
    }

    // Publish pose of the hand as a TF for RVIZ
    {
        geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(kdlf_hand_base_pose);
        tf_transform.header.stamp    = ros::Time::now();
        tf_transform.header.frame_id =   tf::resolve("", "world");
        tf_transform.child_frame_id  = tf::resolve("", "kdlf_hand_base_pose");
        br->sendTransform(tf_transform);
    }

    // Publish pose of the hand as a TF for RVIZ

    if (1) {
        geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(kdlf_endeff_pos_7_link);

        tf_transform.header.stamp    = ros::Time::now();
        tf_transform.header.frame_id =   tf::resolve("", "world");
        tf_transform.child_frame_id  = tf::resolve("", "kdlf_endeff_pos");

        tf::vectorEigenToMsg(v_cart_pos_tracked, tf_transform.transform.translation);
        tf::quaternionEigenToMsg(Eigen::Quaterniond(v_cart_orient_tracked), tf_transform.transform.rotation);

        br->sendTransform(tf_transform);
    }


    // / --------------------------------------------------------------------
    // / == Publish markers for Rviz: contact points

    if ( b_publish_virtual_contact_frames_markers && (i_counter % 25 == 0) ) {
        visualization_msgs::MarkerArray marker_array = make_contact_marker_array2(M2E_v(v_contact_on_link_allowed), list_links, "world");

        // add average contact normal ---------
        // Pb, position and orientation in the right frames ...
        visualization_msgs::Marker marker_arrow_normal;

        marker_arrow_normal.header.frame_id = "world";
        marker_arrow_normal.header.stamp    = ros::Time();
        marker_arrow_normal.ns              = "my_namespace";
        marker_arrow_normal.id              = 0 + M2E_v(v_contact_on_link_allowed).rows() * 3; // use the next integers
        marker_arrow_normal.type            = visualization_msgs::Marker::ARROW;
        marker_arrow_normal.lifetime        = ros::Duration(0.1);

        marker_arrow_normal.color.a = 1.0;
        marker_arrow_normal.color.r = 0.0;
        marker_arrow_normal.color.g = 0.5;
        marker_arrow_normal.color.b = 0.5;
        geometry_msgs::Point point_start;
        tf::pointKDLToMsg(kdlf_hand_base_pose.p, point_start);
        geometry_msgs::Point point_end = point_start;

        point_end.x += v3_choosen_contact_normal[0] * 0.1;
        point_end.y += v3_choosen_contact_normal[1] * 0.1;
        point_end.z += v3_choosen_contact_normal[2] * 0.1;

        marker_arrow_normal.points.push_back(point_start);
        marker_arrow_normal.points.push_back(point_end);

        marker_arrow_normal.scale.x = 0.005; // shaft diameter
        marker_arrow_normal.scale.y = 0.008; // head diameter
        marker_arrow_normal.scale.z = 0;     // dont specify head length

        // comment it out, not always interesting
        //        marker_array.markers.push_back(marker_arrow_normal);


        //- --------------------------------


        pub_marker_array.publish(marker_array);
    }
    timeTrack("N5: pub markers rviz", b_debug);


    // / --------------------------------------------------------------------
    // / == Ros publisher (DEBUG)
    if (1) {
        my_msgs::customMsg1 myMsg;
        myMsg.contact1 = v_joint_contact_info.GetColumn(0)[1 + 4];
        myMsg.contact2 = v_joint_contact_info.GetColumn(0)[2 + 4];
        myMsg.contact3 = v_joint_contact_info.GetColumn(0)[3 + 4];


        myMsg.temp1 = v_curr_position[8];
        myMsg.temp2 = tracking_torques.norm();           // this is 0
        myMsg.temp3 = impedance_tracking_torques.norm(); // this is 0

        myMsg.torque1 = d_time_for_last_loop  * 1000 * 1;
        myMsg.torque2 = d_time_since_last_loop  * 1000 * 1;

        //    myMsg.torque2 = v_contact_torque[2];
        myMsg.torque3 = v_contact_torque[3];

        myPublisher.publish(myMsg);

        timeTrack("N6: pub my msg", b_debug);
    }


    // / --------------------------------------------------------------------
    // / == Send an array of stuff (POSITIONS)
    if (0) {
        std_msgs::Float32MultiArray my_array;

        for (int i = 0; i < 1; i++) {
            //      for (int i = 0; i < NB_DOF_ARM; i++) {
            //      my_array.data.push_back( v_curr_position(i) );

            //      my_array.data.push_back( v_desired_pos_rad_Filtered(i) );
            //      my_array.data.push_back( v_desired_arm_pos_filtered(i) );

            //      my_array.data.push_back( v_desired_pos_rad(i) );
            //      my_array.data.push_back( v_arm_desired_pos(i) );
        }
        my_array.data.push_back( v_additionnal_torques.Norm() );
        my_array.data.push_back( v_contact_torque.Norm() );
        my_array.data.push_back( v_virtual_contact_torque_new.Norm() );
        my_array.data.push_back( v_spacem_torques.Norm() );
        my_array.data.push_back( tot_torques.norm() );


        pub_array.publish(my_array);

        timeTrack("N7: pub positions and other", b_debug);
    }

    // / --------------------------------------------------------------------
    // / Publish hand state (real), for RVIZ display after TF conversion

    if (1) {
        if (i_counter % 50 == 0) {
            allegro_hand_state_info.header.stamp = ros::Time::now();

            for (int i = 0; i < NB_DOF_HAND; i++) {
                allegro_hand_state_info.effort[i]   = v_total_torque_a[i];
                allegro_hand_state_info.velocity[i] = 0.0;
                allegro_hand_state_info.position[i] = v_curr_position[i + NB_DOF_ARM];
            }
            pub_hand_state.publish(allegro_hand_state_info);
        }
    }
    timeTrack("N8: pub hand state", b_debug);

    // / --------------------------------------------------------------------
    // / Publish full Robot state (for RVIZ)
    // / Used to display the full robot model
    if (1) {
        robot_state_info.header.stamp = ros::Time::now();

        for (int i = 0; i < NB_DOF_TOT; i++) {
            robot_state_info.velocity[i] = v_curr_velocity[i];
            robot_state_info.position[i] = v_curr_position[i];
        }

        // joint torques
        for (int i = 0; i < NB_DOF_ARM; i++) robot_state_info.effort[i] = v_est_joint_torques[i];

        for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) robot_state_info.effort[i] = v_total_torque_a[i - NB_DOF_ARM];

        pub_robot_state.publish(robot_state_info);

        bool b_pub_debug_state = true;
        if(b_pub_debug_state){
            for (int i = 0; i < NB_DOF_TOT; i++) {
                // copy desired vector
                //                robot_state_info.position[i] = desired_planned_joint_position[i];
                //                robot_state_info.position[i] = v_arm_desired_pos[i];
                robot_state_info.position[i] = v_desired_pos_rad_Filtered[i];
            }
            pub_robot_state2.publish(robot_state_info);


        }
    }

    timeTrack("N9: pub full robot state", b_debug);

    // ////////////////////////////////////
    // / Publish a long vector with a lot of information for ravin
    if (false) {
        my_msgs::info_ravin info_ravin_msg;
        info_ravin_msg.header.stamp = ros::Time::now();

        // Allegro_hand
        for (int i = 0; i < NB_DOF_HAND; i++) {
            info_ravin_msg.allegro_position.push_back(v_curr_position[i + NB_DOF_ARM]);
            info_ravin_msg.allegro_velocity.push_back(v_curr_velocity[i + NB_DOF_ARM]);
            info_ravin_msg.allegro_torques.push_back(v_total_torque_a[i]);
        }


        // Lwr arm
        for (int i = 0; i < NB_DOF_ARM; i++) {
            info_ravin_msg.lwr_position.push_back(v_curr_position[i]);
            info_ravin_msg.lwr_velocity.push_back(v_curr_velocity[i]);
            info_ravin_msg.lwr_estimated_joint_torques.push_back(v_est_joint_torques[i]);
            info_ravin_msg.lwr_measured_joint_torques.push_back(v_curr_torques[i]);
        }

        for (int i = 0; i < 6; i++) info_ravin_msg.lwr_estimated_external_ee_ft.push_back(v_est_endeff_torques[i]);


        // Task info
        info_ravin_msg.task_rotation_rad = d_ravin_task_rotation;


        pub_info_ravin.publish(info_ravin_msg);
    }
    timeTrack("N10: pub long vec for ravin", b_debug);


    ossT << "Time for my loop: " << (mClock.GetTime() - d_lasttime_loopstart) * 1000 << "ms" << endl;

    // ////////////////////////////////////
    // / Publish time debug
    if (1) {
        if (i_counter % 100 == 0) {
            std_msgs::String s_time_debug;

            //            s_time_debug.data = ossT.str();
            //                  s_time_debug.data = tasklist_permanent.oss_task_debug.str();
            s_time_debug.data = oss_tasks.str();

            //                        cout << oss_task_debug.flush();

            s_time_debug.data += ossDebug.str();
            pub_string_debug.publish(s_time_debug);
        }
    }
    if (0) {
        std_msgs::String s_time_debug;

        s_time_debug.data = "";
        s_time_debug.data += ossDebug.str();
        if(!s_time_debug.data.compare(""))
            pub_string_debug.publish(s_time_debug);
    }

    timeTrack("N10: pub debug", b_debug);


    // / End of publisher
    // / --------------------------------------------------------------------


    b_init = 0;
    i_counter++;

    if (i_no_torque > 0) i_no_torque--;
    timeTrack("P", b_debug);

    //  ossG << "Contact level: " << i_contact_level << ": joints [";
    //  for (auto& j: contact_levels[i_contact_level]) {
    //    ossG << j << ", ";
    //  }
    //  ossG << "]\n";


    // / Save the time necessary for the loop
    ossG << "Time for my loop: " << (mClock.GetTime() - d_lasttime_loopstart) * 1000 << "ms" << endl;
    d_lasttime_loopend = mClock.GetTime();

    if ( (mClock.GetTime() - d_lasttime_loopstart) > 0.003 ) {
        ossG << " === [Overtime] ===" << endl;
        ossT << " === [Overtime] ===" << endl;
    }

    timeTrack("Q", b_debug);

    return STATUS_OK;
}

RobotInterface::Status RobotHapticControllerClass::RobotUpdateCore() {
    return STATUS_OK;
}

std::vector<string>split(const string& s,
                         char          delim) {
    stringstream ss(s);
    string item;

    vector<string> tokens;

    while ( getline(ss, item, delim) ) {
        tokens.push_back(item);
    }
    return tokens;
}

void RobotHapticControllerClass::RespondToConsoleCommandByTopic(const std_msgs::StringConstPtr& _msg) {
    std_msgs::String my_message;

    // Cut the message into pieces
    std::vector<std::string> x = split(_msg->data, ' ');

    if ( x.size() ) {
        // get first element and remove it
        string cmd = x.at(0);
        x.erase( x.begin() );

        // get the rest as arguments
        vector<string> args = x;

        RespondToConsoleCommand(cmd, args);
        my_message.data = GetConsole()->GetLines().back();
    } else {
        b_paused = !b_paused;

        // Pause the simulation
        if (b_paused) my_message.data = "Pausing ...";
        else my_message.data = "... UnPausing";

        GetConsole()->Print(""); // Just print something empty to flush the previous response
    }
    pub_pyconsole.publish(my_message);
}

int RobotHapticControllerClass::RespondToConsoleCommand(const string          cmd,
                                                        const vector<string>& args) {
    if (cmd == "ro") {        /* Reset tekscan offset */
        b_reset_offset = 1;
    } else if (cmd == "gr") { /* Only gravity compensation*/
        b_only_gravity_compensation = 1;
    } else if (cmd == "bnp") {
        b_no_projection = !b_no_projection;
        char txt[256];
        sprintf(txt, "b_no_projection is %s", b_no_projection ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "bpin") {
        b_finger_pd_in_nullspace = !b_finger_pd_in_nullspace;
        char txt[256];
        sprintf(txt, "b_pd_in_nullspace is %s", b_finger_pd_in_nullspace ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "bpl") {
        b_follow_planner_joint_position = !b_follow_planner_joint_position;
        if(b_follow_planner_joint_position){
            d_time_start_trajectory = ros::Time::now().toSec(); // set start of planned trajectory to now.
            // reset filter
            cdd_desired_joint_pos_arm->SetState( v_curr_position.GetSubVector(0, NB_DOF_ARM), v_curr_velocity.GetSubVector(0, NB_DOF_ARM) );
        }else{
            // reset cart target as current position
            v_cart_pos_tracked    = v_cart_endeff_pos;
            v_cart_orient_tracked = v_cart_endeff_orient;

            // Set the values of the filter too. Is this enough ??
            cdd_cartesian_position_motion->SetState( E2M_v(v_cart_endeff_pos) );

            //    cdd_cartesian_orientation_motion->SetState( E2M_v( v_cart_endeff_orient.eulerAngles(2, 1, 2) ) );
            MathLib::Vector vec(4);
            vec(0) = Eigen::Quaterniond(v_cart_endeff_orient).w();
            vec(1) = Eigen::Quaterniond(v_cart_endeff_orient).x();
            vec(2) = Eigen::Quaterniond(v_cart_endeff_orient).y();
            vec(3) = Eigen::Quaterniond(v_cart_endeff_orient).z();
            cdd_cartesian_orientationQ_motion->SetState(vec);
        }
        char txt[256];
        sprintf(txt, "b_follow_planner_joint_position is %s", b_follow_planner_joint_position ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "gt") { /* Not only gravity compensation*/
        b_only_gravity_compensation = 0;
    } else if (cmd == "setp") {
        double newValue = atof( args[0].c_str() );
        newValue = TRUNC(newValue, 0.05, 5.0);
        char txt[256];
        sprintf(txt, "new Pressure is %f", newValue);
        GetConsole()->Print(txt);
        d_desired_normal_pressure = newValue;
    } else if (cmd == "setthreshold") { /* contact threshold*/
        double newValue = atof( args[0].c_str() );
        newValue = TRUNC(newValue, 1.0, 200.0);
        char txt[256];
        sprintf(txt, "new Threshold is %f", newValue);
        GetConsole()->Print(txt);

        for (int i = 0; i < full_patch_list.size(); i++) {
            full_patch_list[i]->setThreshold(newValue);
        }
    } else if (cmd == "setvforce") { /* Not only gravity compensation*/
        double newValue = atof( args[0].c_str() );
        newValue = TRUNC(newValue, 0.001, 50.0);
        char txt[256];
        sprintf(txt, "new virtual force is %f", newValue);
        GetConsole()->Print(txt);
        d_desired_virtual_normal_velocity = newValue;
    } else if (cmd == "setcounter") { /* Not only gravity compensation*/
        int newValue = atoi( args[0].c_str() );
        newValue = TRUNC(newValue, 1.0, 20.0);
        char txt[256];
        sprintf(txt, "new Counter Threshold is %i", newValue);
        GetConsole()->Print(txt);
        tekpatch_set_1->setPatchesContactCounter(newValue);
    }
    else if (cmd == "dnam") {
        char txt[256];
        sprintf( txt, "new name is %s", args[0].c_str() );
        GetConsole()->Print(txt);
        strcpy( mSaveDataName, args[0].c_str() );
    } else if (cmd == "p") {
        //    mSaveDataNumber++;
        //    char txt[256];
        //    sprintf(txt, "new DataNumber is %i", mSaveDataNumber);
        //    GetConsole()->Print(txt);
        b_ravin_close_mode        = false;
        v_ravin_joint_pos_tracked = M2E_v(v_curr_position);

        // not enough ...
    }
    else if (cmd == "bt") {
        b_auto_pos_tracking = !b_auto_pos_tracking;
        char txt[256];
        sprintf(txt, "Using b_auto_pos_tracking: %s", b_auto_pos_tracking ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "ba") {
        b_auto_next_point = !b_auto_next_point;
        char txt[256];
        sprintf(txt, "Using b_auto_next_point: %s", b_auto_next_point ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "bs") {
        b_auto_orientation = !b_auto_orientation;
        char txt[256];
        sprintf(txt, "Using b_auto_orientation: %s", b_auto_orientation ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "kg") {
        ctrlm_robot_next = GRAV_COMP;
        cout << "Ok, Control mode set to Grav Comp" << endl;
    } else if (cmd == "kt") {
        v_cart_pos_tracked    = v_cart_endeff_pos;
        v_cart_orient_tracked = v_cart_endeff_orient;

        // Set the values of the filter too. Is this enough ??
        cdd_cartesian_position_motion->SetState( E2M_v(v_cart_endeff_pos) );

        //    cdd_cartesian_orientation_motion->SetState( E2M_v( v_cart_endeff_orient.eulerAngles(2, 1, 2) ) );
        MathLib::Vector vec(4);
        vec(0) = Eigen::Quaterniond(v_cart_endeff_orient).w();
        vec(1) = Eigen::Quaterniond(v_cart_endeff_orient).x();
        vec(2) = Eigen::Quaterniond(v_cart_endeff_orient).y();
        vec(3) = Eigen::Quaterniond(v_cart_endeff_orient).z();
        cdd_cartesian_orientationQ_motion->SetState(vec);

        //    v_cart_pos_tracked_filtered = v_cart_pos_tracked;
        //    v_cart_orient_tracked_filtered = v_cart_endeff_orient;

        ctrlm_robot_next       = TORQUE;
        v_last_position_static = v_curr_position;

        cout << "Ok, Control mode set to Torque" << endl;
    } else if (cmd == "ks") {
        ctrlm_robot_next = POSITION_PID_STATIC;
        cout << "Ok, Control mode set to position static" << endl;
    }

    else if (cmd == "setkp") {
        double newValue = atof( args[0].c_str() );
        newValue = TRUNC(newValue, 0.0001, 5.0);
        char txt[256];
        sprintf(txt, "new kp value is %f", newValue);
        GetConsole()->Print(txt);
        d_joint_position_kp = newValue;
    } else if (cmd == "setkd") {
        double newValue = atof( args[0].c_str() );
        newValue = TRUNC(newValue, 0.0001, 5.0);
        char txt[256];
        sprintf(txt, "new kd value is %f", newValue);
        GetConsole()->Print(txt);
        d_joint_position_kd = newValue;
    }

    //    else if (cmd == "kp") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kp value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_cartesian.SetKP(MathLib::Vector(3).One()*newValue);
    //    }else if (cmd == "kd") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kd value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_cartesian.SetKD(MathLib::Vector(3).One()*newValue);
    //    }else if (cmd == "kI") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kd value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_cartesian.SetKI(MathLib::Vector(3).One()*newValue);
    //    }
    else if (cmd == "gd") {
        //    b_arm_for_virtual_force = !b_arm_for_virtual_force;
        //    char txt[256];
        //    sprintf(txt, "Using b_arm_for_virtual_force: %s", b_arm_for_virtual_force ? "true" : "false");
        //    GetConsole()->Print(txt);

        //    b_ravin_get_a_direction = true;

        v3_dir_p0_to_p1.Set( (list_links[i_side_1_major]->_kdl_frame.p - list_links[i_side_0_major]->_kdl_frame.p).data );
        v3_dir_p0_to_p1.Normalize();
        ROS_INFO_STREAM( "direction:" << M2E_v(v3_dir_p0_to_p1) );
    } else if (cmd == "vc") {
        b_use_virtual_contact_torques = !b_use_virtual_contact_torques;
        char txt[256];
        sprintf(txt, "Using b_use_virtual_contact_torques: %s", b_use_virtual_contact_torques ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "bc") {
        b_temp_switch = !b_temp_switch;

        //    b_compensate_coriolis = !b_compensate_coriolis;
        char txt[256];
        sprintf(txt, "Using b_temp_switch: %s", b_temp_switch ? "true" : "false");

        //    sprintf(txt, "Using Coriolis Torques: %s", b_compensate_coriolis ? "true" : "false");
        GetConsole()->Print(txt);
    }

    else if (cmd == "l") {
        int newValue = atoi( args[0].c_str() );

        //          setContactLevel(newValue);
        setContactLevel2(newValue, contact_levels);
    }

    //    else if (cmd == "sc") {
    //        int i_dir=atoi(args[0].c_str());
    //        double d_dist=atof(args[1].c_str());
    //        d_dist=TRUNC(d_dist,-1.0,1.0);
    //        char txt[256];
    //        sprintf(txt, "Move %f in direction %i",d_dist,i_dir );
    //        GetConsole()->Print(txt);
    //        SetTraj(d_dist,i_dir);
    //        if(args.size()>2)
    //            d_cart_vel_desired=atof(args[2].c_str());
    //    }
    //    else if (cmd == "kpj") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kp value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_joint.SetKP(MathLib::Vector(NB_DOF_TOT).One()*newValue);
    //    }else if (cmd == "kdj") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kd value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_joint.SetKD(MathLib::Vector(NB_DOF_TOT).One()*newValue);
    //    }else if (cmd == "kIj") {
    //        double newValue=atof(args[0].c_str());
    //        newValue=TRUNC(newValue,0.0001,100.0);
    //        char txt[256];
    //        sprintf(txt, "new kd value is %f",newValue );
    //        GetConsole()->Print(txt);
    //        pid_controller_joint.SetKI(MathLib::Vector(NB_DOF_TOT).One()*newValue);
    else if (cmd == "k") {
        double newValue = atof( args[1].c_str() );
        newValue = TRUNC(newValue, 0.0001, 1000.0);
        char txt[256];
        sprintf(txt, "new k? value is %f", newValue);
        GetConsole()->Print(txt);

        // / For Stiffness control
        if (args[0] == "p") {
            myImpedanceController->SetStiffnessPosition_e(newValue); // not a lot .. //800 //20
            sprintf(txt, "Setting stiffness newValue %f", newValue);
            GetConsole()->Print(txt);
        } else if (args[0] == "d") {
            myImpedanceController->SetDampingPosition_e(newValue); // 80 //20 //100
            sprintf(txt, "Setting damping newValue");
            GetConsole()->Print(txt);
        } else if ( !std::strcmp(args[0].c_str(), "po") ) {
            myImpedanceController->SetStiffnessOrientation_e(newValue); // not a lot .. //800 //20
            sprintf(txt, "Setting stiffness orientation newValue %f", newValue);
            GetConsole()->Print(txt);
        } else if ( !std::strcmp(args[0].c_str(), "do") ) {
            myImpedanceController->SetDampingOrientation_e(newValue); // 80 //20 //100
            sprintf(txt, "Setting damping orientation newValue %f", newValue);
            GetConsole()->Print(txt);
        } else if ( !std::strcmp(args[0].c_str(), "dj") ) {
            sprintf(txt, "Setting joint damping coefficient to %f", newValue);
            GetConsole()->Print(txt);

            double d_joint_damping_coefficient = 1.0;
            double temp_joint_damping;

            for (int i = 0; i < NB_DOF_TOT; i++) {
                temp_joint_damping = m_jsim_tot(i, i) * d_joint_damping_coefficient;
                myImpedanceController->SetJointSpaceDamping_e(i, temp_joint_damping);
            }
        }
    }
    else if (cmd == "des") {
        double x = atof( args[0].c_str() );
        double y = atof( args[1].c_str() );
        double z = atof( args[2].c_str() );
        e_final_pos_exploration << x, y, z;
    } else if (cmd == "g") { // set the desired cart position from the list
        int index = atoi( args[0].c_str() );

        if ( (index > -1) && ( index < stdvec_des_cart_positions.size() ) ) {
            e_final_pos_exploration = stdvec_des_cart_positions[index];
            i_curr_despos_i         = index;
        }
    }

    else if (cmd == "n") { // set the desired cart position from the list
        if ( (i_curr_despos_i + 1) < stdvec_des_cart_positions.size() ) {
            i_curr_despos_i++;
        } else {
            i_curr_despos_i = 0;
        }

        e_final_pos_exploration = stdvec_des_cart_positions[i_curr_despos_i];
    }  else if (cmd == "bcj") {
        b_use_joint_centering = !b_use_joint_centering;
        char txt[256];
        sprintf(txt, "Centering joints: %s", b_use_joint_centering ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "bjl") {
        b_avoid_joint_limits = !b_avoid_joint_limits;
        char txt[256];
        sprintf(txt, "Avoiding joint limits: %s", b_avoid_joint_limits ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "gi") {
        b_gazebo_object_imp_zero = !b_gazebo_object_imp_zero;
        char txt[256];
        sprintf(txt, "b_gazebo_object_imp_zero: %s", b_gazebo_object_imp_zero ? "true" : "false");
        GetConsole()->Print(txt);

        if (!b_gazebo_object_imp_zero) {
            //      SendGazeboImpedanceValues(150.0, 3.0, 5.0, 0.3, true);
            SendGazeboImpedanceValues(1000.0, 3.0, 50.0, 0.3, true); // very stiff
        }
        else {
            SendGazeboImpedanceValuesFree();
        }
    } else if (cmd == "go") {
        // move by 10 cm with gaussian noise sigma=1cm
        //    std::default_random_engine generator;
        //    std::normal_distribution<double> distribution(0.1, 0.01);
        //    double sample = distribution(generator);
        //    ROS_WARN_STREAM("sample: " << sample);
        b_ravin_jointpos_reach = false;
        v_cart_pos_tracked     = v_cart_endeff_pos;

        v_cart_pos_tracked[2] -= 0.1;

        //    v_cart_pos_tracked[2] -= sample;
        v_cart_orient_tracked = v_cart_endeff_orient;
        b_ravin_cart_reach    = true;
    } else if (cmd == "gb") {
        v_cart_pos_tracked[2] += 0.1;
        b_ravin_cart_reach     = false;
        b_close_allowed        = false;
    } else if (cmd == "pause") {
        b_paused = !b_paused;
        char txt[256];
        sprintf(txt, "pause: %s", b_paused ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "A1") {
        b_disableA1 = !b_disableA1;
        char txt[256];
        sprintf(txt, "b_disableA1: %s", b_disableA1 ? "true" : "false");
        GetConsole()->Print(txt);
    } else if (cmd == "Aeq1") {
        b_disableAeq1 = !b_disableAeq1;
        char txt[256];
        sprintf(txt, "b_disableAeq1: %s", b_disableAeq1 ? "true" : "false");
        GetConsole()->Print(txt);
    }


    return 0;
}

extern "C" {
// These two "C" functions manage the creation and destruction of the class
RobotHapticControllerClass* create() {
    return new RobotHapticControllerClass();
}

void destroy(RobotHapticControllerClass *module) {
    delete module;
}
}


// //////////////////////////////////////////////////////////////////////////////

bool RobotHapticControllerClass::readTekscan(bool resetOffset) {
    bool pendingReads = tactileTekscanPort.getPendingReads();

    if (pendingReads) {
        // Get some timing info
        // Get the data, filter it, put in a Matrix
        yarp::os::Bottle *input             = tactileTekscanPort.read(false);
        yarp::os::Bottle *tekscanDataBottle = input->get(1).asList();

        bFilterTekscan = 0; // / 0

        for (int i = 0; i < TEKSCAN_SIZE; i++) {
            if (!bFilterTekscan) {
                tekscanData[i] = tekscanDataBottle->get(i).asDouble();                                                       // no, as int ??? pb ...
            } else {
                tekscanData[i] += ( -tekscanData[i] + tekscanDataBottle->get(i).asDouble() ) * 0.02 / mTekscanDataFilterTau; /* Replace this def*/
            }
        }

        if (resetOffset) {
            tekscanDataOffset = tekscanData; /* maybe do that before filtering, NO */
            b_reset_offset    = 0;
        }

        tekscanData -= tekscanDataOffset;
        tekscanData.Trunc(0.0, 256.0);

        tekscanDataMatrix->Set(tekscanData.Array(), TEKSCAN_ROW, TEKSCAN_COL); // pb: set as an array of double, they are ints ...

        // FAKE IT !!!
        if (b_simulate_tekscan) {
            tekscanDataMatrix->Ref(5, 16) = 100.0;
        }

        // Parse the data
        tekpatch_set_1->setFullDataMatrix(tekscanDataMatrix);

        // Here, parse the other matrix simply ??


        return 1;
    } else {
        return 0;
    }
}

// /////////////////////////////////////////////
bool RobotHapticControllerClass::readTekscanDual(bool         resetOffset,
                                                 TekPatchSet *set1,
                                                 TekPatchSet *set2) {
    bool pendingReads = tactileTekscanPort.getPendingReads();

    if (pendingReads) {
        // Get some timing info
        // Get the data, filter it, put in a Matrix
        yarp::os::Bottle *input             = tactileTekscanPort.read(false);
        yarp::os::Bottle *tekscanDataBottle = input->get(1).asList();

        int i_tekscan_length;
        int i_2handles_tekscan_columns = (TEKSCAN_COL * 2 + 1);


        if (tekscanDataBottle->size() == i_2handles_tekscan_columns * TEKSCAN_ROW) {
            // Two handles
            i_tekscan_length = i_2handles_tekscan_columns * TEKSCAN_ROW;
        } else {
            cerr << " wrong size of tekscan (expecting two handles, no calibration data): " << tekscanDataBottle->size() << endl;
            ROS_FATAL("Wrong size of tekscan");

            exit(0);
        }


        tekscanData.Resize(i_tekscan_length);
        tekscanDataOffset.Resize(i_tekscan_length);


        for (int i = 0; i < i_tekscan_length; i++) {
            tekscanData[i] = tekscanDataBottle->get(i).asDouble();
        }

        if (resetOffset) {
            tekscanDataOffset = tekscanData; // maybe do that before filtering, NO
            b_reset_offset    = 0;
        }

        tekscanData -= tekscanDataOffset;
        tekscanData.Trunc(0.0, 256.0);


        MathLib::Matrix tekscanDataMatrixFull_;
        MathLib::Matrix tekscanDataMatrix1_;
        MathLib::Matrix tekscanDataMatrix2_;

        tekscanDataMatrixFull_.Set(tekscanData.Array(), TEKSCAN_ROW, i_2handles_tekscan_columns);
        tekscanDataMatrix1_.Set( tekscanDataMatrixFull_.GetColumns(0, TEKSCAN_COL - 1) );
        tekscanDataMatrix2_.Set( tekscanDataMatrixFull_.GetColumns(TEKSCAN_COL + 1, TEKSCAN_COL * 2) );

        set1->setFullDataMatrix(&tekscanDataMatrix1_);
        set2->setFullDataMatrix(&tekscanDataMatrix2_);


        return 1;
    } else {
        return 0;
    }
}

void RobotHapticControllerClass::jacobianSpe1RowPerContactFromLinkID_e(std::vector<TactileContact *>& list_links,
                                                                       int                            cid,
                                                                       Mat                          & tempJacobian3_e) {
    // TODO in case it's not a full 7+4 vector !!!
    // Get the rotation matrix from the link to the world (to express the Jacobian in the link frame intead of the world frame)
    Mat R_root_2_link_stdmtrix_e = Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> >(list_links[cid]->_kdl_frame.M.data, 3, 3);

    R_root_2_link_stdmtrix_e.transposeInPlace(); // =R_Hand_2_Link_stdmtrix_e.transpose();

    // Get first 3 rows of the contact Jacobian (only position) // WRONG ...
    tempJacobian3_e = list_links[cid]->_kld_contact_jacobian.data.topRows(3);

    // Rotate this Jacobian to the link Frame (TODO: actually, it should be rotated in the frame of the task, not necessarily the link ...)
    tempJacobian3_e = R_root_2_link_stdmtrix_e * tempJacobian3_e;

    // Get the direction of the normal
    Vec normal_e = M2E_v(list_links[cid]->normalFromLink);
    tempJacobian3_e = normal_e.transpose() * tempJacobian3_e;
}

Mat RobotHapticControllerClass::jacobianSpe1RowPerContactFromLinkID_e(std::vector<TactileContact *>& list_links,
                                                                      int                            cid) {
    Mat tempJacobian3_e;

    jacobianSpe1RowPerContactFromLinkID_e(list_links, cid, tempJacobian3_e);

    return tempJacobian3_e;
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::NullSpaceThread_c() {
    while (1)
    {
        //        cerr << "inside nullspaceThread .." < < endl;
        {
            // Need to be inside a scope otherwise the following lock gets stuck
            std::unique_lock<std::mutex> lock(bm_mutex_main_c);

            while (b_new_Jacobian_c == false) {
                bc_condition_c.wait(lock); // got to sleep
            }

        }
        //        cerr << "XXX passed lock for nullspaceThread" << endl;
        //ope space controller
        OperationalSpaceControl ope_space_controller_local;
        {
            std::lock_guard<std::mutex> guard_c(bm_mutex_main_c);
            ope_space_controller_local = ope_space_controller_permanent;
        }
        // nullspace controller
        TaskList tasklist_local;
        {
            // Protect the copy of the TaskList
            std::lock_guard<std::mutex> guard_c(bm_mutex_main_c);
            tasklist_local = tasklist_permanent;
        }

        Vec temp_torques = Vec::Zero( tasklist_local._JSIM.rows() );

        //                bool   b_use_qp   = false;
        //        bool   b_use_qp   = true;
        double time_start = mClock.GetTime();

        if (!b_use_qp) {
            temp_torques = tasklist_local.getTotalTorque();
        } else {
            temp_torques = ope_space_controller_local.computeTorquesQP();
        }

        tasklist_local.oss_task_debug << "time to finish: " << mClock.GetTime() - time_start << endl;
        ope_space_controller_local.s_task_debug += "time to finish: " + to_string(mClock.GetTime() - time_start) + "\n";

        {
            std::lock_guard<std::mutex> guard_c(bm_mutex_main_c);

            if(!temp_torques.allFinite()){
                // print tasklist_local
                // do it with qp ?
                //                ROS_FATAL_STREAM("tasklist_local._JSIM: " << tasklist_local._JSIM);
                //                ROS_FATAL_STREAM("tasklist_local.oss_task_debug: " << tasklist_local.oss_task_debug.str());
                //                ROS_FATAL_STREAM("tasklist_local.oss_task_debug: " << tasklist_local.mmap_task);



                ROS_FATAL_STREAM("temp_torques: " << temp_torques);
                temp_torques.setZero();
                //                exit(0);
            }
            tot_torques_permanent = temp_torques;

            b_new_Jacobian_c = false;
            oss_tasks.str("");
            oss_tasks << tasklist_local.oss_task_debug.str();
            oss_tasks << ope_space_controller_local.s_task_debug;
        }
    }
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::timeTrack(string text,
                                           bool   b_err) {
    std::ostringstream oss_temp;

    oss_temp.str("");
    mClock.Update();
    double delta_time = (mClock.GetTime() - lastTime);
    oss_temp << text << "\t. Time passed(ms): " << delta_time * 1000;
    double barsize_in_us = 5.0;
    int    numberofbars  = (delta_time * 1000000 / barsize_in_us);
    oss_temp << std::string(numberofbars, '-').c_str();

    if (delta_time > 0.0002) {
        oss_temp << " [Overtime!]";
    }

    oss_temp << endl;


    // Send to cerr (for crash debugs) or to normal time stream
    if (b_err) {
        ROS_DEBUG_STREAM( oss_temp.str() );
        cerr << oss_temp.str();
    } else {
        ossT << oss_temp.str();
    }


    lastTime = mClock.GetTime();
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::RavinCallbackTrajectory(const sensor_msgs::JointStateConstPtr& _msg)
{
    // If controlling the real robot
    if (ctrlm_robot != POSITION_PID) {
        ctrlm_robot_next = POSITION_PID;

        cdd_desired_joint_pos_arm->SetState( v_curr_position.GetSubVector(0, NB_DOF_ARM), v_curr_velocity.GetSubVector(0, NB_DOF_ARM) );
        // I may have to replace this line with the real velocity of the state...
        cdd_desired_joint_pos_hand->SetState( v_curr_position.GetSubVector(NB_DOF_ARM, NB_DOF_HAND), Vector(NB_DOF_HAND).Zero() );
    }

    // why do we test this ?
    if (!b_ravin_jointpos_reach) {
        //      if (!b_ravin_jointpos_reach || 1) {
        cdd_desired_joint_pos_hand->SetState( v_curr_position.GetSubVector(NB_DOF_ARM, NB_DOF_HAND), Vector(NB_DOF_HAND).Zero() );

        b_ravin_jointpos_reach = true;
        i_no_torque            = 5;     // dont send torque in the next few iterations (security against transition ...)
        b_ravin_close_mode     = false; // not so useful.
        b_ravin_grasped_mode   = false;
        b_ravin_task_mode      = false;
        PublishState("reaching");
    }
    GetConsole()->Print("Getting new trajectory info 2");

    if ( this->v_ravin_joint_pos_tracked.rows() == _msg->position.size() ) this->v_ravin_joint_pos_tracked = Eigen::Map<const VectorXd>( _msg->position.data(), _msg->position.size() );
    else cerr << "Pb: receiving jointState with wrong length" << endl;

    d_last_time_trajectory = mClock.GetTime();
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::RavinCallbackMode(const my_msgs::ModeConstPtr& _msg)
{
    b_ravins_mode = _msg->mode; // This SHOULD be renamed ...
    v_contact_on_link_allowed.Zero();


    b_close_signal = false;

    // patch 1
    double highest_pressure = 0;

    ossGv << "side 1:" << endl;

    // "normal" patch
    for (int i = 0; i < NB_DOF_HAND; i++) {
        int i_cor = i + NB_DOF_ARM;

        if (_msg->patch1[i] > 0) {
            list_links[i_cor]->i_side = 0;

            //      list_links[i_cor]->d_desired_pressure = _msg->patch1[i] * d_ravin_patch_pressure_mutliplier;
            list_links[i_cor]->d_desired_pressure = _msg->patch1[i];
            v_contact_on_link_allowed(i_cor)      = 1.0;

            if (highest_pressure < _msg->patch1[i]) {
                highest_pressure     = _msg->patch1[i];
                this->i_side_0_major = i_cor;
            }
        }
    }

    // "side" patch: (only 12 patches)
    for (int i = 0; i < NB_DOF_FINGER * 3; i++) {
        int i_cor = i + NB_DOF_ARM;

        if (_msg->patch1[i + NB_DOF_HAND] > 0) {
            list_links[i_cor]->i_side = 0;

            //      list_links[i_cor]->d_desired_pressure = _msg->patch1[i + NB_DOF_HAND] * d_ravin_patch_pressure_mutliplier;
            list_links[i_cor]->d_desired_pressure = _msg->patch1[i + NB_DOF_HAND];
            v_contact_on_link_allowed(i_cor)      = 1.0;

            if (highest_pressure < _msg->patch1[i + NB_DOF_HAND]) {
                highest_pressure     = _msg->patch1[i + NB_DOF_HAND];
                this->i_side_0_major = i_cor;
            }
        }
    }


    highest_pressure = 0;
    ossGv << "side 2:" << endl;

    // / PATCH 2
    // "normal" patch
    for (int i = 0; i < NB_DOF_HAND; i++) {
        int i_cor = i + NB_DOF_ARM;

        if (_msg->patch2[i] > 0) {
            list_links[i_cor]->i_side             = 1;
            list_links[i_cor]->d_desired_pressure = _msg->patch2[i];

            //      list_links[i_cor]->d_desired_pressure = _msg->patch2[i] * d_ravin_patch_pressure_mutliplier;
            v_contact_on_link_allowed(i_cor) = 1.0;

            //      cerr << "front link :" << i <<  endl;

            if (highest_pressure < _msg->patch2[i]) {
                highest_pressure     = _msg->patch2[i];
                this->i_side_1_major = i_cor;
            }
        }
    }

    // "side" patch: (only 12 patches)
    for (int i = 0; i < NB_DOF_FINGER * 3; i++) {
        int i_cor = i + NB_DOF_ARM;

        if (_msg->patch2[i + NB_DOF_HAND] > 0) {
            list_links[i_cor]->i_side             = 1;
            list_links[i_cor]->d_desired_pressure = _msg->patch2[i + NB_DOF_HAND];

            //      list_links[i_cor]->d_desired_pressure = _msg->patch2[i + NB_DOF_HAND] * d_ravin_patch_pressure_mutliplier;
            v_contact_on_link_allowed(i_cor) = 1.0;

            //      cerr << "side link :" << i <<  endl;

            if (highest_pressure < _msg->patch2[i + NB_DOF_HAND]) {
                highest_pressure     = _msg->patch2[i + NB_DOF_HAND];
                this->i_side_1_major = i_cor;
            }
        }
    }

    //  exit(0);
}

// ///////////////////////////////////////////////////////
void RobotHapticControllerClass::RavinCallbackTaskCartesianRotation(const my_msgs::TaskConstPtr& _msg)
{
    // Here I should receive an axis-angle and rotation velocity
    b_task_mode_signal = true;


    if ( !_msg->task_type.data.compare("rotation") ) {
        ROS_WARN("Got a new rotation task: ");

        b_task_rotation  = true;
        b_task_hammering = false;

        task_rotation->lwr_rotation_axis(0) = _msg->direction.x;
        task_rotation->lwr_rotation_axis(1) = _msg->direction.y;
        task_rotation->lwr_rotation_axis(2) = _msg->direction.z;

        task_rotation->lwr_rotation_point(0) = _msg->position.x;
        task_rotation->lwr_rotation_point(1) = _msg->position.y;
        task_rotation->lwr_rotation_point(2) = _msg->position.z;

        task_rotation->lwr_rotation_velocity = _msg->velocity.data;
    }

    if ( !_msg->task_type.data.compare("hammering") ) {
        ROS_WARN("Got a new hammering task: ");
        b_task_rotation  = false;
        b_task_hammering = true;


        task_hammering->ravin_hammering_direction(0) = _msg->direction.x;
        task_hammering->ravin_hammering_direction(1) = _msg->direction.y;
        task_hammering->ravin_hammering_direction(2) = _msg->direction.z;
        task_hammering->d_amplitude                  = d_temp_1;
        task_hammering->d_time_period                = d_temp_2;
    }
}

// ///////////////////////////////////////////////////////////////
void RobotHapticControllerClass::RavinCallbackEmptyClose(const std_msgs::EmptyConstPtr& _msg)
{
    ROS_WARN("received close signal");
    b_close_signal          = true;
    b_close_allowed         = true;
    b_ravin_get_a_direction = true;

    //  b_ravin_cart_reach = true;
}

// ///////////////////////////////////////////////////////////////
void RobotHapticControllerClass::RavinCallbackEmptyTighten(const std_msgs::EmptyConstPtr& _msg)
{
    b_tighten_signal = true;
}

// ///////////////////////////////////////////////////////////////
void RobotHapticControllerClass::AudioRecognitionCallback(const std_msgs::StringConstPtr& _msg)
{
    ROS_INFO( "I heard: [%s]", _msg->data.c_str() );


    if (_msg->data.c_str() == "gravity compensation on") b_only_gravity_compensation = 1;

    if (_msg->data.c_str() == "gravity compensation off") b_only_gravity_compensation = 0;
}

// ///////////////////////////////////////////////////////////////
void RobotHapticControllerClass::GetCommandString(const std_msgs::StringConstPtr& _msg)
{
    if ( !_msg->data.compare("stop") ) {
        b_ravin_jointpos_reach = false;
        b_ravin_close_mode     = false;
        b_ravin_grasped_mode   = false;
        b_ravin_task_mode      = false;
        b_task_hammering       = false;
        b_task_rotation        = false;

        v_cart_pos_tracked    = v_cart_endeff_pos;
        v_cart_orient_tracked = v_cart_endeff_orient;

        // Also reset desired cartesian position ...
    }


    if ( !_msg->data.compare("go") ) {
        // Also switch mode ???
        b_ravin_jointpos_reach = false;
        v_cart_pos_tracked     = v_cart_endeff_pos;
        v_cart_pos_tracked[2] -= 0.1;

        v_cart_orient_tracked = v_cart_endeff_orient;
        b_ravin_cart_reach    = true;

        // save the "object's" position for grasping
        v_cart_pose_object = Eigen::Translation3d(v_cart_pos_tracked) * Eigen::Quaterniond(v_cart_endeff_orient);

        b_ravin_close_mode = false;
        b_close_signal     = false;
    }


    if ( !_msg->data.compare("gb") ) {
        v_cart_pos_tracked[2] += 0.1;
        b_ravin_cart_reach     = false;
        b_close_allowed        = false;
        b_ravin_close_mode     = false;
        b_close_signal         = false;
    }

    if ( !_msg->data.compare("open") ) {
        b_close_signal         = false;
        b_ravin_close_mode     = false;
        b_ravin_cart_reach     = false;
        b_close_allowed        = false;
        b_ravin_jointpos_reach = false;
        v_cart_pos_tracked     = v_cart_endeff_pos;
    }
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::CallbackGraspingPerturbation(const std_msgs::Float32MultiArrayConstPtr& _msg)
{
    // Decide which position to base myself on...
    v_cart_pos_tracked     = v_cart_pose_object.translation();
    v_cart_pos_tracked[0] += _msg.get()->data[0];
    v_cart_pos_tracked[1] += _msg.get()->data[1];
    v_cart_pos_tracked[2] += _msg.get()->data[2];

    // Decide which orientation to base myself on
    Vector3d ea = v_cart_pose_object.rotation().eulerAngles(0, 1, 2);

    ea(0) += _msg.get()->data[3];
    ea(1) += _msg.get()->data[4];
    ea(2) += _msg.get()->data[5];

    v_cart_orient_tracked = AngleAxisd( ea(0), Vector3d::UnitX() )
            * AngleAxisd( ea(1),  Vector3d::UnitY() )
            * AngleAxisd( ea(2), Vector3d::UnitZ() ); // This is made to take degrees ?
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::UpdateSpaceNavDataTwist(const geometry_msgs::Twist::ConstPtr& _msg) {
    this->twist_msg_.linear.x  = _msg->linear.x;
    this->twist_msg_.linear.y  = _msg->linear.y;
    this->twist_msg_.linear.z  = _msg->linear.z;
    this->twist_msg_.angular.x = _msg->angular.x;
    this->twist_msg_.angular.y = _msg->angular.y;
    this->twist_msg_.angular.z = _msg->angular.z;
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::UpdateSpaceNavDataJoy(const sensor_msgs::Joy::ConstPtr& _msg) {
    this->joy_msg_.buttons = _msg->buttons;
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::UpdateGazeboJointState(const std_msgs::Float32MultiArrayConstPtr& _msg) {
    int i = 0;

    for (std::vector<float>::const_iterator it = _msg->data.begin(); it != _msg->data.end(); ++it) {
        v_curr_position_tempgaz[i] = *it;
        i++;
    }
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::CallbackAllegroJointState(const sensor_msgs::JointState& _msg) {
    for (int i = 0; i < NB_DOF_HAND; i++) v_position_a[i] = _msg.position[i];
}

// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::UpdateSimContactState(const gazebo_msgs::ContactsState::ConstPtr& _msg) {
    this->stdvec_gazebo_contacts.clear();
    this->stdvec_gazebo_contacts = _msg->states;
}
// //////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::RobotTrajectoryCallBack(const moveit_msgs::RobotTrajectoryConstPtr &_msg)
{
    // Copy the first position.
    b_got_trajectory = true;
    planned_trajectory = *_msg.get();


}

void RobotHapticControllerClass::HandConfigCallback(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    for (int i=0; i <_msg.get()->data.size() ; i++){
        v_hand_explo_posture[i] = _msg.get()->data[i];
    }
    b_got_hand_explo_posture = true;

}

void RobotHapticControllerClass::HandStateCallback(const  sensor_msgs::JointState&_msg)
{
    for (int i = 0; i < NB_DOF_HAND; i++) v_hand_explo_posture[i] = _msg.position[i];
    b_got_hand_explo_posture = true;


}

// //////////////////////////////////////////////////////////////////////////////
// / custom callback queue thread (NOT SURE HOW THIS IS USEFUL)
void RobotHapticControllerClass::QueueThread() {
    static const double timeout = 0.01;

    while ( this->rosnode_->ok() ) {
        this->queue_.callAvailable( ros::WallDuration(timeout) );
    }
}

// //////////////////////////////////////////////////////////////////////////////
// void RobotHapticControllerClass::UpdateJacobianAndFk(TactileContact *contact_list, int i_nb_of_contacts, bool b_debug, bool b_contact_rotation) {
void RobotHapticControllerClass::UpdateJacobianAndFk(std::vector<TactileContact *>& contact_list,
                                                     int                            i_nb_of_contacts,
                                                     bool                           b_debug,
                                                     bool                           b_contact_rotation) {
    for (int i = 0; i < i_nb_of_contacts; i++) {
        //      timeTrack("E20", true);

        // First check if the contact is valid:
        if ( v_contact_on_link_allowed[i] || (contact_list[i]->contact_status != NO_CONTACT) ) {
            //        cerr << "contact in the loop ..." << i << endl;
            //        ros::Time timestart = ros::Time::now();

            // Get the kinematic Chain
            this->my_tree.getChain("world", contact_list[i]->s_link_name_urdf, contact_list[i]->kdlchain_kinchain);

            // Add the segment corresponding to the contact in the link frame
            KDL::Rotation kdlrot_normal_rotation; // Create a rotation, only the normal is important (z), but I need to fix the two others:
            KDL::Vector   kdlvec_link_to_contact_normal = KDL::Vector( contact_list[i]->normalFromLink.x(), contact_list[i]->normalFromLink.y(), contact_list[i]->normalFromLink.z() );
            KDL::Vector   kdlvec_tan1;

            if (abs( kdlvec_link_to_contact_normal.x() ) + abs( kdlvec_link_to_contact_normal.y() ) > 0.001) {         // if the vector is not unitZ
                kdlvec_tan1 = KDL::Vector(kdlvec_link_to_contact_normal.y(), -kdlvec_link_to_contact_normal.x(), 0.0); // Create a normal vector
            } else {
                kdlvec_tan1 = KDL::Vector(kdlvec_link_to_contact_normal.z(), 0.0, 0.0);
            }

            kdlvec_tan1.Normalize();

            KDL::Vector kdlvec_tan2 = kdlvec_link_to_contact_normal * kdlvec_tan1;
            kdlvec_tan2.Normalize();

            //      cerr << "time until now (01): " << ros::Time::now() - timestart << endl;

            // Fill the rotation matrix
            kdlrot_normal_rotation.UnitX(kdlvec_tan1);
            kdlrot_normal_rotation.UnitY(kdlvec_tan2);
            kdlrot_normal_rotation.UnitZ(kdlvec_link_to_contact_normal);

            if (!b_contact_rotation) { // if not contact rotation, reset to have the link rotation
                kdlrot_normal_rotation = kdlrot_normal_rotation.Identity();
            }

            //      cerr << "time until now (02): " << ros::Time::now() - timestart << endl;

            KDL::Vector kdlvec_link_to_contact_pos = KDL::Vector(contact_list[i]->v_position_from_link[0], contact_list[i]->v_position_from_link[1], contact_list[i]->v_position_from_link[2]);
            KDL::Frame  kdlframe_link_to_contact   = KDL::Frame(kdlrot_normal_rotation, kdlvec_link_to_contact_pos);
            contact_list[i]->kdlchain_kinchain.addSegment( KDL::Segment(KDL::Joint(KDL::Joint::None), kdlframe_link_to_contact) );

            // Create the chainSolver for getting the Jacobian and the Fwd Kin Solver for getting the transformation to the link
            KDL::ChainJntToJacSolver kdlcjsolver_contactsolver    = KDL::ChainJntToJacSolver(contact_list[i]->kdlchain_kinchain);
            KDL::ChainFkSolverPos_recursive kdlfksolver_fk_solver = KDL::ChainFkSolverPos_recursive(contact_list[i]->kdlchain_kinchain);


            // Create the Joint positions vector
            int i_chain_length = contact_list[i]->kdlchain_kinchain.getNrOfJoints();

            if (!i_chain_length) {
                cerr << "Link " << contact_list[i]->s_link_name_urdf << endl;
                cerr << "Chain length is 0" << endl;
                ROS_FATAL("Chain length is 0");
                exit(0);
            }

            //      cerr << "time until now (03): " << ros::Time::now() - timestart << endl;

            KDL::JntArray kdlja_joint_vector = KDL::JntArray(i_chain_length);

            for (int j = 0; j < min(NB_DOF_ARM, i_chain_length); j++) { // fill in arm positions until end of chain
                kdlja_joint_vector(j) = v_curr_position[j];
            }

            // Fill in finger positions until end of chain
            if (i_chain_length > NB_DOF_ARM) {
                int finger = (contact_list[i]->i_dof_n - NB_DOF_ARM) / 4;

                for (int j = NB_DOF_ARM; j < min(NB_DOF_ARM + NB_DOF_FINGER, i_chain_length); j++) {
                    kdlja_joint_vector(j) = v_curr_position[j + 4 * finger];
                }
            }

            // Compute the Jacobian
            contact_list[i]->_kld_contact_jacobian.resize(i_chain_length); // should be resized before !! (without this it caused big troubles)
            kdlcjsolver_contactsolver.JntToJac(kdlja_joint_vector, contact_list[i]->_kld_contact_jacobian);

            //      cerr << "time until now (04): " << ros::Time::now() - timestart << endl;

            kdlfksolver_fk_solver.JntToCart(kdlja_joint_vector, contact_list[i]->_kdl_frame);


            if ( (i > 5) && false ) {
                ossGv << "Link n " << i << endl;
                ossGv << "contact_list[i]->_kld_ contact_jacobian: \n" << contact_list[i]->_kld_contact_jacobian.data << endl;
                ossGv << "Contact's position : " << contact_list[i]->_kdl_frame.p.x() << ":" << contact_list[i]->_kdl_frame.p.y() << ":" << contact_list[i]->_kdl_frame.p.z() << endl;
                ossGv << "Contact's normal: " << contact_list[i]->_kdl_frame.M.UnitZ().x() << ":" << contact_list[i]->_kdl_frame.M.UnitZ().y() << ":" << contact_list[i]->_kdl_frame.M.UnitZ().z() << endl;
                ossGv << "position from link: \n " << M2E_v(contact_list[i]->v_position_from_link).transpose() << endl;
                ossGv << "link frame: \n" << Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> >(list_links[i]->_kdl_frame.M.data, 3, 3) << endl;
                ossGv << "normal from link: \n" << M2E_v(list_links[i]->normalFromLink) << endl;
            }


            // Remove the segment that was previously added.
            contact_list[i]->kdlchain_kinchain.segments.pop_back(); // Ugly but works (but Segment number variable now stays wrong in the chain...)
            //      cerr << "time until now (05): " << ros::Time::now() - timestart << endl;

            if (b_debug) {
                ossGv << "Virtual contact on link " << i << endl;
                ossGv << "Contact's position : " << contact_list[i]->_kdl_frame.p.x() << ":" << contact_list[i]->_kdl_frame.p.y() << ":" << contact_list[i]->_kdl_frame.p.z() << endl;

                //        int finger = (contact_list[i]->i_dof_n - NB_DOF_ARM) / 4; // check that mDofN is set ...

                //        cerr << "---------===---------" << endl;
                //        cerr << "Debug contact N_" << i << endl;
                //        cerr << "Link name:" << contact_list[i]->s_link_name_urdf << endl; /// Why does this crash ??
                //        cerr << "i_chain_length:" << i_chain_length << endl;

                //        cerr << "finger:" << finger << endl;
                //        cerr << "contact_list[i]->mDofN:" << contact_list[i]->i_dof_n << endl;
                //        cerr << "contact_list[i]->kdlchain_kinchain.getNrOfJoints(): " << contact_list[i]->kdlchain_kinchain.getNrOfJoints() << endl;
                //        cerr << "Contact Jacobian:\n" << contact_list[i]->_kld_contact_jacobian.data << endl;
            }


            //      ros::Time timestart = ros::Time::now();
            //      cerr << "time until now (end of loop): " << ros::Time::now() - timestart << endl;
        }
    }
}

void RobotHapticControllerClass::setContactLevel2(int                           level,
                                                  std::vector<std::vector<int> >contact_levels) {
    // Force the level to be between bounds.
    if (level < 0) level = 0;

    if ( level >= contact_levels.size() ) level = contact_levels.size() - 1;
    i_contact_level = level;

    char txt[256];
    sprintf(txt, "new Contact level is %i", level);
    GetConsole()->Print(txt);

    v_contact_on_link_allowed.Zero();

    for (auto& j : contact_levels[level]) {
        // Add the arm's length
        v_contact_on_link_allowed(j + NB_DOF_ARM) = 1;
    }
}

// /////////////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::SetTraj(double distance,
                                         int    direction) {
    //  Eigen::Map<Eigen::Matrix<double, 3, 1> > v_cart_endeff_pos(kdlf_endeff_pos.p.data);
    v_cart_desired_pos_final             = v_cart_endeff_pos;
    v_cart_desired_pos_final(direction) += distance;

    v_cart_desired_pos_tracked = v_cart_endeff_pos;

    b_reaching_trajectory = true;
}

// ///////////////////////////////////////////////////////////////////////////////////////
void RobotHapticControllerClass::Compute_JSIM() {
    // Prepare the joint position arrays .. no better way to do that ? /// and the velocities: to be tested ...
    for (int i = 0; i < NB_DOF_ARM; i++) { // fill in arm positions
        kdl_jntarray_f0(i) = v_curr_position[i];
        kdl_jntarray_f1(i) = v_curr_position[i];
        kdl_jntarray_f2(i) = v_curr_position[i];
        kdl_jntarray_f3(i) = v_curr_position[i];

        kdl_jntarray_vel_f0(i) = v_curr_velocity_filtered_CDD[i];
        kdl_jntarray_vel_f1(i) = v_curr_velocity_filtered_CDD[i];
        kdl_jntarray_vel_f2(i) = v_curr_velocity_filtered_CDD[i];
        kdl_jntarray_vel_f3(i) = v_curr_velocity_filtered_CDD[i];
    }

    for (int i = NB_DOF_ARM; i < NB_DOF_ARM + NB_DOF_FINGER; i++) { // fill in each finger's position
        kdl_jntarray_f0(i) = v_curr_position[i];
        kdl_jntarray_f1(i) = v_curr_position[i + 4];
        kdl_jntarray_f2(i) = v_curr_position[i + 8];
        kdl_jntarray_f3(i) = v_curr_position[i + 12];

        kdl_jntarray_vel_f0(i) = v_curr_velocity_filtered_CDD[i];
        kdl_jntarray_vel_f1(i) = v_curr_velocity_filtered_CDD[i + 4];
        kdl_jntarray_vel_f2(i) = v_curr_velocity_filtered_CDD[i + 8];
        kdl_jntarray_vel_f3(i) = v_curr_velocity_filtered_CDD[i + 12];
    }

    /* Compute JSIMs */
    kdlcdp_f0->JntToMass(kdl_jntarray_f0, jointspace_inertia_f0);
    kdlcdp_f1->JntToMass(kdl_jntarray_f1, jointspace_inertia_f1);
    kdlcdp_f2->JntToMass(kdl_jntarray_f2, jointspace_inertia_f2);
    kdlcdp_f3->JntToMass(kdl_jntarray_f3, jointspace_inertia_f3);

    /* Compute Coriolis forces */
    kdlcdp_f0->JntToCoriolis(kdl_jntarray_f0, kdl_jntarray_vel_f0, kdl_jntarray_coriolis_f0);
    kdlcdp_f1->JntToCoriolis(kdl_jntarray_f1, kdl_jntarray_vel_f1, kdl_jntarray_coriolis_f1);
    kdlcdp_f2->JntToCoriolis(kdl_jntarray_f2, kdl_jntarray_vel_f2, kdl_jntarray_coriolis_f2);
    kdlcdp_f3->JntToCoriolis(kdl_jntarray_f3, kdl_jntarray_vel_f3, kdl_jntarray_coriolis_f3);


    for (int i = 0; i < NB_DOF_ARM + NB_DOF_FINGER; i++) {
        for (int j = 0; j < NB_DOF_ARM + NB_DOF_FINGER; j++) {
            mat_JSIM_f0(i, j) = jointspace_inertia_f0(i, j);
            mat_JSIM_f1(i, j) = jointspace_inertia_f1(i, j);
            mat_JSIM_f2(i, j) = jointspace_inertia_f2(i, j);
            mat_JSIM_f3(i, j) = jointspace_inertia_f3(i, j);
        }
    }

    // Construct the Coriolis torques vector
    v_torques_coriolis.Zero();
    v_torques_coriolis.SetSubVector( 0,               MathLib::Vector(kdl_jntarray_coriolis_f0.data.data(), NB_DOF_ARM + NB_DOF_FINGER) );
    v_torques_coriolis.SetSubVector( NB_DOF_ARM + 4,  MathLib::Vector(kdl_jntarray_coriolis_f1.data.segment(4, 4).data(), NB_DOF_FINGER) );
    v_torques_coriolis.SetSubVector( NB_DOF_ARM + 8,  MathLib::Vector(kdl_jntarray_coriolis_f2.data.segment(4, 4).data(), NB_DOF_FINGER) );
    v_torques_coriolis.SetSubVector( NB_DOF_ARM + 12, MathLib::Vector(kdl_jntarray_coriolis_f3.data.segment(4, 4).data(), NB_DOF_FINGER) );

    // Construct the whole matrix (tested, works ok)
    m_jsim_tot.Zero();

    // arm + finger 0
    m_jsim_tot.InsertSubMatrix(0, 0, mat_JSIM_f0, 0, NB_DOF_ARM + NB_DOF_FINGER, 0, NB_DOF_ARM + NB_DOF_FINGER);

    //        m_jsim_tot.InsertSubMatrix(0,               0,               mat_JSIM_f0, 0,          NB_DOF_ARM , 0,          NB_DOF_ARM); // only finger
    //        m_jsim_tot.InsertSubMatrix(NB_DOF_ARM,  NB_DOF_ARM,  mat_JSIM_f0, NB_DOF_ARM, NB_DOF_FINGER,              NB_DOF_ARM, NB_DOF_FINGER); // finger 0

    // finger 1
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 4,  0,               mat_JSIM_f1, NB_DOF_ARM, NB_DOF_FINGER, 0,          NB_DOF_ARM);    // finger 1 bottom
    m_jsim_tot.InsertSubMatrix(0,               NB_DOF_ARM + 4,  mat_JSIM_f1, 0,          NB_DOF_ARM,    NB_DOF_ARM, NB_DOF_FINGER); // finger 1 right
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 4,  NB_DOF_ARM + 4,  mat_JSIM_f1, NB_DOF_ARM, NB_DOF_FINGER, NB_DOF_ARM, NB_DOF_FINGER); // finger 1 bottom-right

    // finger 2
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 8,  0,               mat_JSIM_f2, NB_DOF_ARM, NB_DOF_FINGER, 0,          NB_DOF_ARM);    // finger 1 bottom
    m_jsim_tot.InsertSubMatrix(0,               NB_DOF_ARM + 8,  mat_JSIM_f2, 0,          NB_DOF_ARM,    NB_DOF_ARM, NB_DOF_FINGER); // finger 1 right
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 8,  NB_DOF_ARM + 8,  mat_JSIM_f2, NB_DOF_ARM, NB_DOF_FINGER, NB_DOF_ARM, NB_DOF_FINGER); // finger 1 bottom-right

    // finger 3
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 12, 0,               mat_JSIM_f3, NB_DOF_ARM, NB_DOF_FINGER, 0,          NB_DOF_ARM);    // finger 1 bottom
    m_jsim_tot.InsertSubMatrix(0,               NB_DOF_ARM + 12, mat_JSIM_f3, 0,          NB_DOF_ARM,    NB_DOF_ARM, NB_DOF_FINGER); // finger 1 right
    m_jsim_tot.InsertSubMatrix(NB_DOF_ARM + 12, NB_DOF_ARM + 12, mat_JSIM_f3, NB_DOF_ARM, NB_DOF_FINGER, NB_DOF_ARM, NB_DOF_FINGER); // finger 1 bottom-right
    //  }
}

// ///////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d RobotHapticControllerClass::FrameFromNormal(Eigen::Vector3d contact_normal,
                                                            Vector3d        contact_normal2,
                                                            int             axis,
                                                            bool            next_axis) {
    Eigen::Matrix3d normal_Frame;

    Eigen::Vector3d tan1, tan2, tan_temp;

    if (contact_normal.norm() < 0.0001) { // If vector is null, force it to something before normalization
        contact_normal(0) = 1.0;
    }

    contact_normal.normalize();

    // Create tan1 as projection of contact_normal2 on orthogonal space of contact_normal
    tan_temp = contact_normal2 - contact_normal2.dot(contact_normal) * contact_normal;

    if (tan_temp.norm() < 0.0001) { // If vector is null, force it to something before normalization
        tan_temp(1) = 1.0;
    }

    tan_temp.normalize();

    if (next_axis) {
        tan1 = tan_temp;
        tan2 = contact_normal.cross(tan1);
        tan2.normalize();
    } else {
        tan2 = tan_temp;
        tan1 = tan1.cross(contact_normal);
        tan1.normalize();
    }


    // Depending on which axis is the normal, define the matrix
    switch (axis) {
    case 0:
        normal_Frame << contact_normal, tan1, tan2;
        break;

    case 1:
        normal_Frame << tan2, contact_normal, tan1;
        break;

    case 2:
        normal_Frame <<  tan1, tan2, contact_normal;
        break;
    }

    return normal_Frame;
}

// ///////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d RobotHapticControllerClass::FrameFromNormal(Eigen::Vector3d contact_normal,
                                                            int             axis) {
    Eigen::Vector3d tan1, tan2;
    Eigen::Matrix3d normal_Frame;

    if (contact_normal.norm() < 0.0001) { // If vector is null, force it to something before normalization
        contact_normal(0) = 1.0;
    }

    contact_normal.normalize();

    if (abs( contact_normal(0) ) + abs( contact_normal(1) ) > 0.001) { // if the vector is not unitZ
        tan1 << contact_normal(1), -contact_normal(0), 0.0;
    } else {
        tan1 << contact_normal(2), 0.0, 0.0;
    }

    tan1.normalize();
    tan2 = contact_normal.cross(tan1);
    tan2.normalize();

    // Depending on which axis is the normal, define the matrix
    switch (axis) {
    case 0:
        normal_Frame << contact_normal, tan1, tan2;
        break;

    case 1:
        normal_Frame << tan2, contact_normal, tan1;
        break;

    case 2:
        normal_Frame <<  tan1, tan2, contact_normal;
        break;
    }

    return normal_Frame;
}

MathLib::Matrix RobotHapticControllerClass::get_full_vector_from_short(MathLib::Matrix shortv,
                                                                       unsigned int    finger) {
    MathLib::Matrix fullv(shortv.RowSize(), NB_DOF_TOT);

    // Copy arm part
    fullv.SetColumnSpace(shortv.GetColumns(0, NB_DOF_ARM - 1), 0);

    // Copy finger part if existant
    if (finger >= 0) {
        fullv.SetColumnSpace(shortv.GetColumns(NB_DOF_ARM, shortv.ColumnSize() - 1), NB_DOF_ARM + finger * NB_DOF_FINGER);
    }

    return fullv;
}

Mat RobotHapticControllerClass::get_full_vector_from_short_e(Mat          shortv,
                                                             unsigned int finger) {
    Mat fullv = Mat::Zero(shortv.rows(), NB_DOF_TOT);

    int nb_cols = shortv.cols();

    // Copy arm part
    fullv.leftCols( min(nb_cols, NB_DOF_ARM) ) = shortv.leftCols( min(nb_cols, NB_DOF_ARM) );

    //  fullv.leftCols(NB_DOF_ARM) = shortv.leftCols(NB_DOF_ARM);

    // Copy finger part if existant
    if ( (shortv.cols() >= NB_DOF_ARM) && (finger >= 0) && (finger < NB_FINGERS) ) {
        fullv.block(0, NB_DOF_ARM + finger * NB_DOF_FINGER, shortv.rows(), shortv.cols() - NB_DOF_ARM) = shortv.rightCols(shortv.cols() - NB_DOF_ARM);
    }
    return fullv;
}

void RobotHapticControllerClass::SendGazeboImpedanceValues(double lin_stiff,
                                                           double rot_stiff,
                                                           double lin_damp,
                                                           double rot_damp,
                                                           bool   b_reset_position) {
    std_msgs::Float32MultiArray my_impedance_coeffs_array;

    my_impedance_coeffs_array.data.clear();

    for (int i = 0; i < NB_DOF_TOT; i++) {
        my_impedance_coeffs_array.data.push_back(lin_stiff);
        my_impedance_coeffs_array.data.push_back(rot_stiff);
        my_impedance_coeffs_array.data.push_back(lin_damp);
        my_impedance_coeffs_array.data.push_back(rot_damp);
        my_impedance_coeffs_array.data.push_back(b_reset_position ? 1.0 : 0.0); // RESET
    }
    pub_gazebo_object_impedance.publish(my_impedance_coeffs_array);
}

void RobotHapticControllerClass::SendGazeboImpedanceValuesFree() {
    SendGazeboImpedanceValues(0.0, 0.0, 0.0, 0.0, false);
}

void RobotHapticControllerClass::PublishState(std::string str) {
    std_msgs::String task_state;

    task_state.data = str;

    if ( str.compare(last_state_sent) ) {
        pub_task_state.publish(task_state);
        last_state_sent = str;
    }
}

void RobotHapticControllerClass::callback(my_msgs::my_dyn_paramsConfig& config,
                                          uint32_t                      level) {
    // Dynamic parameters is to debug something.
    // During normal runs, this code should not be ran (risk of overwrite of the good parameters)

    d_impedance_lin_stiff                 = config.d_impedance_lin_stiff;
    d_impedance_lin_damp                  = config.d_impedance_lin_damp;
    d_impedance_rot_damp                  = config.d_impedance_rot_damp;
    d_impedance_rot_stiff                 = config.d_impedance_rot_stiff;
    d_use_vc_normalinfo                   =   config.d_use_vc_normalinfo;
    d_max_torque_arm_centering            =   config.d_max_torque_arm_centering;
    d_min_jointlimit_distance_percent_arm =   config.d_min_jointlimit_distance_percent_arm;
    d_jsim_regularization =   config.d_jsim_regularization;

    d_ravin_patch_pressure_mutliplier = config.d_touch_pressure_mult;
    b_use_identity_JSIM               = config.b_use_identity_JSIM;

    d_temp_5               = config.d_temp_5;
    d_temp_6               = config.d_temp_6;
    b_use_qp               = config.b_use_qp;

    //  b_handonly_nullspace_next         = config.b_handonly_nullspace;
    d_desired_normal_pressure = config.d_desired_normal_pressure;
    d_desired_virtual_normal_velocity = config.d_desired_virtual_normal_velocity;
    //    cout << "d_desired_normal_pressure after dynparam: " << d_desired_normal_pressure << endl;

    b_finger_pd_in_nullspace = config.b_pd_in_nullspace;
    d_ns_torques_multiplier = config.d_ns_torques_multiplier;
    d_grav_comp_multiplier = config.d_grav_comp_multiplier;

    //    exit(0);

    // Only get some parameters after the program is running
    if (i_counter > 10) {
        d_vc_cart_kp = config.d_vc_cart_kp;

        for (int i_linkn = 0; i_linkn < NB_DOF_TOT; i_linkn++) {
            list_links[i_linkn]->pid_controller_cart.SetKP(MathLib::Vector(3).One() * d_vc_cart_kp);
        }

        d_vc_cart_kd = config.d_vc_cart_kd;

        for (int i_linkn = 0; i_linkn < NB_DOF_TOT; i_linkn++) {
            list_links[i_linkn]->pid_controller_cart.SetKD(MathLib::Vector(3).One() * d_vc_cart_kd);
        }

        // Update impedance parameters
        myImpedanceController->SetStiffnessPosition_e(config.d_impedance_lin_stiff);
        myImpedanceController->SetDampingPosition_e(config.d_impedance_lin_damp);
        myImpedanceController->SetStiffnessOrientation_e(config.d_impedance_rot_stiff);
        myImpedanceController->SetDampingOrientation_e(config.d_impedance_rot_damp);

        myImpedanceController->SetJointSpaceDamping_e(config.d_joint_damping_coefficient);
    }
}
