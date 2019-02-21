/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL,
 *Switzerland
 * Author: Nicolas Sommer

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

#ifndef RobotHapticController_H_
#define RobotHapticController_H_

//#include "tasks_nico.h"
#include "ope_space_controller.h"
#include "ImpedanceController.h"

#include <kdl_parser/kdl_parser.hpp> // read kdl trees from urdf
// files/parameters, needs to be loaded
// before mathlib because PI config
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include "RobotLib/RobotInterface.h"
#include "RobotLib/KinematicChain.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/PIDController.h"

// #include "controlAllegroHand.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

#include <dynamic_reconfigure/server.h>

// dynamic reconfigure
#include "my_msgs/my_dyn_paramsConfig.h"


//#include "MotionGenerators/CDDynamics.h"
#include "CDDynamics.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "moveit_msgs/RobotTrajectory.h"

// #include "nico_haptic_package/customMsg1.h" // My own message
#include "my_msgs/customMsg1.h" // My own message
#include "my_msgs/Mode.h"       // My own message
#include "my_msgs/Task.h"       // My own message
#include "my_msgs/info_ravin.h" // My own message


#include "KUKARobotModel/LWRRobot.h"

#include "Eigen/Core"
#include "Eigen/Dense"


#include "joint_limits_interface/joint_limits.h"
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <pcl_ros/point_cloud.h>

#include <tf/transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_kdl/tf2_kdl.h"
#include "kdl_conversions/kdl_msg.h"

#include <array>

#include "tuple"


using Eigen::Map;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


typedef Eigen::VectorXd Vec;
typedef Eigen::MatrixXd Mat;


// New for receiving some stuff through ros topic

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>


#include <ros/ros.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelState.h>

#include "tekPatch.h"

using namespace Eigen;
//using namespace yarp::os;


#include "publisher_helpers.h"
#include "mathlib_eigen_conversions.h"
#include "virtual_contact_helpers.h"
#include "task_ravin.h"
#include "robot_haptic_controller_helper.h"

#include "eigen_conversions/eigen_kdl.h"
#include "eigen_conversions/eigen_msg.h"


#define NB_DOF_HAND 16
#define NB_DOF_ARM 7
#define NB_DOF_TOT NB_DOF_ARM + NB_DOF_HAND
#define NB_FINGERS 4
#define NB_DOF_FINGER 4


#include "globals.h"


class RobotHapticControllerClass : public RobotInterface {
  public:

    RobotHapticControllerClass();

    virtual ~RobotHapticControllerClass();

    virtual Status RobotInit();

    virtual Status RobotFree();

    virtual Status RobotStart();

    virtual Status RobotStop();

    virtual Status RobotUpdate();

    virtual Status RobotUpdateCore();

    virtual int RespondToConsoleCommand(const string cmd,
                                        const vector<string> &args);


    RevoluteJointSensorGroup mSensorsGroup;
    RevoluteJointActuatorGroup mActuatorsGroup;

    //    Vector                      jointAngles;

    int nBaseID;
    int nHandID2;
    int nIndexID;
    int baseOfChain;

    int nbContacts_old;
    int i_nb_contacts;

    bool b_paused;
    bool b_disableA1=false;
    bool b_disableAeq1=false;

    MathLib::IndicesVector stdvec_indices_finger[4];

    MathLib::Vector v_curr_position;
    MathLib::Vector v_curr_torques;
    MathLib::Vector v_curr_position_tempgaz;
    MathLib::Vector v_curr_position_actuator;
    MathLib::Vector v_initial_position; // This initial position does not contain
    // AllegroHand Position
    MathLib::Vector v_prev_position;

    MathLib::Vector curr_position_filtered_CDD;

    MathLib::Vector v_curr_velocity;
    MathLib::Vector v_prev_velocity;

    Vec v_current_velocity_filtered_2deg;
    Vec v_current_position_filtered_2deg;

    //    MathLib::Vector             curr_velocity_filtered;
    MathLib::Vector v_curr_velocity_filtered_CDD;
    MathLib::Vector v_curr_velocity_filtered_CDD2;
    MathLib::Vector v_prev_velocity_filtered_CDD;

    //    MathLib::Vector             curr_acceleration_filtered;
    MathLib::Vector v_curr_acceleration_filtered_CDD1;
    MathLib::Vector v_curr_acceleration_filtered_CDD2;
    MathLib::Vector v_prev_desired_pos_rad_Filtered;
    MathLib::Vector v_desired_velocity;

    MathLib::Vector v_curr_torque_a; // just to fill the getjointinfo function
    MathLib::Vector v_position_a;
    MathLib::Vector v_position_a_old;

    sensor_msgs::JointState allegro_hand_state_desired_torque;
    sensor_msgs::JointState allegro_hand_state_info;
    sensor_msgs::JointState robot_state_info;

    bool b_no_projection;

    // Allegro gains
    // Default parameters.
    double k_p[NB_DOF_HAND] =
    {
      // Default P Gains for PD Controller, loaded if
      // 'gains_pd.yaml' file is not loaded.
      600.0, 600.0, 600.0, 1000.0, 600.0, 600.0, 600.0, 1000.0,
      600.0, 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 600.0
    };

    double k_d[NB_DOF_HAND] =
    {
      // Default D Gains for PD Controller, loaded if
      // 'gains_pd.yaml' file is not loaded.
      15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,
      15.0, 20.0, 15.0, 15.0, 30.0, 20.0, 20.0, 15.0
    };

    std::string pGainParams[NB_DOF_HAND] =
    {
      "my_gains_pd/p/j00", "my_gains_pd/p/j01", "my_gains_pd/p/j02",
      "my_gains_pd/p/j03",
      "my_gains_pd/p/j10", "my_gains_pd/p/j11", "my_gains_pd/p/j12",
      "my_gains_pd/p/j13",
      "my_gains_pd/p/j20", "my_gains_pd/p/j21", "my_gains_pd/p/j22",
      "my_gains_pd/p/j23",
      "my_gains_pd/p/j30", "my_gains_pd/p/j31", "my_gains_pd/p/j32",
      "my_gains_pd/p/j33"
    };

    std::string dGainParams[NB_DOF_HAND] =
    {
      "my_gains_pd/d/j00", "my_gains_pd/d/j01", "my_gains_pd/d/j02",
      "my_gains_pd/d/j03",
      "my_gains_pd/d/j10", "my_gains_pd/d/j11", "my_gains_pd/d/j12",
      "my_gains_pd/d/j13",
      "my_gains_pd/d/j20", "my_gains_pd/d/j21", "my_gains_pd/d/j22",
      "my_gains_pd/d/j23",
      "my_gains_pd/d/j30", "my_gains_pd/d/j31", "my_gains_pd/d/j32",
      "my_gains_pd/d/j33"
    };

    //    MathLib::Vector             desire_torque;
    MathLib::Vector v_gravComp_torque;
    MathLib::Vector v_contact_torque;
    MathLib::Vector v_virtual_contact_torques;

    //    MathLib::Vector             *contact_torque_each;

    MathLib::Vector v_additionnal_torques;
    MathLib::Vector v_total_torque;
    MathLib::Vector v_total_torque_a;
    MathLib::Vector v_additionnal_torque_friction;


    MathLib::Vector v_last_position_static;

    Vec v_est_joint_torques;
    Vec v_est_endeff_torques;

    Vector3d lwr_measured_cart_position;
    Matrix3d lwr_measured_cart_orientation;


    ImpedanceController *myImpedanceController;

    //    MathLib::Vector             v_cart_imp_target;

    Eigen::Matrix<double, 3, 1> v_cart_pos_tracked;
    Eigen::Matrix<double, 3, 3> v_cart_orient_tracked;

    Eigen::Affine3d v_cart_pose_object;

    Eigen::Vector3d v_cart_pos_tracked_filtered;
    Eigen::Matrix3d v_cart_orient_tracked_filtered;

    Vector3d v_cart_pos_tracked_ravin;
    Matrix3d v_cart_orient_tracked_ravin;

    bool b_cart_command_ready;
    bool b_finished_task;
    double d_ravin_task_rotation;

    Eigen::Vector3d e_final_pos_exploration;


    bool b_auto_pos_tracking;
    bool b_auto_next_point;

    bool b_use_point_tracking;

    // For a virtual contact, use the closest's contact normal direction
    // (instead of the direction to that contact)
    bool b_use_vc_normalinfo;
    double d_use_vc_normalinfo; // same but now variable between 0 and 1

    bool b_finger_pd_in_nullspace; // Process the PD torques through the nullspace algo or not.
    bool b_follow_planner_joint_position = false; // Process the PD torques through the nullspace algo or not.
    bool b_disable_index = false; // Disable index finger (broken...)

    bool* b_fingers_should_open;
    bool* b_fingers_should_open_old;


    //    char*                        char_v_torques_for_vel_pid;


    double d_task_weight_vc;
    double d_task_weight_space_mouse;
    double d_task_weight_joint_centering;
    double d_task_weight_joint_position;
    double d_task_weight_joint_limit;

    double d_min_jointlimit_distance_percent_arm;
    double d_max_torque_arm_centering;


    bool b_init;
    int i_counter;
    bool b_debug;
    int i_no_torque;

    double mStartTime;
    double dt;

    Vector3 v3_dir_p0_to_p1;

    MathLib::Vector kps;
    MathLib::Vector kds;


    // TORQUE: the joint is used to apply a torque.
    // POSITION_PID: PID reaching a position
    // POSITION_PID_STATIC: keeping current position
    // GRAV_COMP: USED With Kuka joints
    //  enum                        CtrlMode { TORQUE = 0, POSITION_PID,
    //                                         POSITION_PID_STATIC, GRAV_COMP, CART_IMP };
    CtrlMode *jointCtrlMode; // These are arrays
    CtrlMode *prevJointCtrlMode;

    CtrlMode ctrlm_robot;    // These are not arrays
    CtrlMode ctrlm_robot_prev;
    CtrlMode ctrlm_robot_next;

    //  MathLib::Vector3 lwr_cart_position;
    //  MathLib::Matrix3 lwr_cart_orientation;
    //  MathLib::Vector3 lwr_cart_position_init;
    //  MathLib::Matrix3 lwr_cart_orientation_init;

    Vector3d lwr_des_cart_position_e;
    Matrix3d lwr_des_cart_orientation_e;
    Vector3d lwr_cart_position_init_e;
    Matrix3d lwr_cart_orientation_init_e;

    //

    Vector3d lwr_rotation_axis;
    Vector3d lwr_rotation_point;
    double lwr_rotation_velocity;

    Vector3d ravin_hammering_direction;

    Vector computedPhaseShift;
    bool b_was_disabled;

    bool b_arm_for_virtual_force;

    MathLib::Vector v_zeros_on_arm;
    MathLib::Vector v_temp_arm_size;


    MathLib::Vector v_pos_KP;
    MathLib::Vector v_pos_KD;
    MathLib::Vector v_pos_KI;

    double d_pos_KP;
    double d_pos_KD;


    MathLib::Vector v_vel_KP;
    MathLib::Vector v_vel_KD;
    MathLib::Vector v_vel_KI;
    MathLib::Vector v_joint_error;
    MathLib::Vector v_joint_error_prev;
    MathLib::Vector v_joint_error_deriv;
    MathLib::Vector v_joint_error_integral;

    MathLib::Vector v_torques_for_joint_error;

    //  MathLib::Vector v_torque_for_vel_pid;

    //  MathLib::Vector v_velocity_desired;
    //  MathLib::Vector v_velocity_error;


    MathLib::Vector v_desired_pos_rad;
    MathLib::Vector v_desired_pos_rad_Filtered;
    MathLib::Vector v_des_diff_rad_Filtered;


    MathLib::Vector v_desired_hand_pos_filtered;
    MathLib::Vector v_desired_arm_pos_filtered;

    //  bool b_use_kukas_impedance_mode;


    double lastTime;
    double d_lasttime_loopstart;
    double d_lasttime_loopend;
    double d_lasttime_receivedPos;
    double d_lasttime_receivedPos2;
    double d_received_counter;
    double d_received_counter2;
    double lastTimeNullSpace;

    double d_last_time_trajectory;


    bool b_tick;

    double d_last_time_contact_down;
    double d_last_time_for_contact_up;

    Vector mJointMotionAmplitude;
    Vector mJointMotionMean;

    //  Vector mTimePeriod;
    Vector mPhaseShift;
    Vector mPeriodicMinAngle;
    Vector mPeriodicMaxAngle;
    Vector mDesiredPositionDeg;

    double currentTime;
    double startTime;
    double wn;
    Vector mRestPosition; // This intial position contains good AllegroHand
    // position values. Keep the 2 values for now


    MathLib::Matrix mSaveDataMatrix;

    //  bool                        SavingOn;
    unsigned int mSaveDataPos;
    char mSaveDataName[256];
    int mSaveDataNumber;

    MathLib::Vector saveDataRow;


    void timeTrack(string text,
                   bool b_err = false); // Useful to check which part of the
    // code is taking a long time


    Clock mClock;

    /* Tekscan things, yarp, etc */
    bool bUseTekscan;
    bool b_tekscan_con_success;
    yarp::os::BufferedPort<yarp::os::Bottle> tactileTekscanPort;

    bool readTekscan(bool resetOffset = false);

    bool readTekscanDual(bool resetOffset,
                         TekPatchSet *set1,
                         TekPatchSet *set2);

    bool bFilterTekscan;
    Vector tekscanData;
    Vector tekscanDataOffset;
    double mTekscanDataFilterTau;

    //    Matrix                      tekscanDataMatrix;
    MathLib::Matrix *tekscanDataMatrix;

    TekPatchSet *mtekpatches;
    TekPatchSet *tekpatch_set_1;
    TekPatchSet *tekpatch_set_2;

    std::vector<TekPatch *> full_patch_list;

    bool b_reset_offset;
    double d_desired_normal_pressure;         // / Should be defined somewhere else
    double d_desired_virtual_normal_velocity; // / Should be defined somewhere else
    double d_jsim_regularization; // compensate for small weights of the allegroHand

    bool b_handonly_nullspace;
    bool b_handonly_nullspace_next;

    // used when streaming torques to the simulator: just send the hand torques
    bool b_simulate_hand_only;

    bool b_no_thumb_from_opt;

    double d_target_reached_threshold;
    double d_ravin_finger_orient_stiffness;
    double d_angle_sum_max_limit;

    // enable or disable commannds from space mouse buttons
    //(may not be desired if using space mouse for something else)
    bool b_enable_spmouse_buttons;

    // Easily usable variables ...
    bool b_ravin_parrallel_closing;
    bool b_ravin_use_orientation_for_vc;
    bool b_ravin_allow_tighten_switch;
    bool b_ravin_use_fake_mode_data;
    bool b_ravin_auto_close_switch;
    bool b_ravin_bypass_hand_error_check_reach;
    bool b_use_filtered_cartesian_target;

    double d_temp_1;
    double d_vc_cart_kp;
    double d_vc_cart_kd;
    double d_ravin_min_time_for_reach_finished;
    double d_hand_kp_multiplier;
    double d_hand_kd_multiplier;
    double d_ravin_patch_pressure_mutliplier;
    double d_ravin_hand_error_norm_threshold_reach;
    double d_ravin_hand_velocity_threshold_reach;
    double d_ravin_time_auto_mini_tighten_switch;
    double d_ravin_max_time_unscrewing;
    double d_ravin_joint_limit_threshold_unscrewing;
    double d_vc_cart_max_gap;
    double d_temp_2;
    double d_temp_3;
    double d_temp_4;
    double d_temp_5;
    double d_temp_6;
    double d_joint_close_velocity;
    double d_temp_8;
    double d_temp_9;
    double d_temp_10;

    bool b_use_qp;

    bool b_protect_undesired_contacts;

    int i_temp_1;
    int i_temp_2;
    int i_temp_3;

    bool b_display_time;
    bool b_publish_virtual_contact_frames_markers;

    bool b_ravin_get_a_direction;

    MathLib::Vector v_finger_contact_max_dof;

    //  MathLib::Vector v_finger_contact_max_dof_id;
    MathLib::Matrix v_joint_contact_info;

    //    int                         i_contact_on_link_allowed[NB_DOF_TOT];
    MathLib::Vector v_contact_on_link_allowed;


    bool b_use_gazebo_contact_data;

    bool b_use_identity_JSIM;


    char mBaseName[256];


    //    MathLib::Vector             joint_upper_limit;


    //  controlAllegroHand *canDevice;
    bool b_allegro_success;
    bool b_allegro_connected_once;
    bool b_allegro_data_micro_change;
    bool b_use_allegro = false;


    int i_a_hand_counter;

    bool b_use_gazebo;
    bool b_control_real_robot;
    bool b_use_predetermined_pd_gains;

    InverseDynamics mInvDynamics;


    bool b_only_gravity_compensation;
    bool b_use_projected_Torques;

    bool b_compensate_coriolis;
    bool b_temp_switch;

    //  CDDynamics *cdd_desiredRadPos;
    CDDynamics *cdd_desired_joint_pos_arm;
    CDDynamics *cdd_desired_joint_pos_hand;
    Vector velLimits;
    CDDynamics *cdd_positionFilter;
    CDDynamics *cdd_velocityFilter;
    CDDynamics *cdd_contact_normal_filter;
    CDDynamics *cdd_max_intensity;

    CDDynamics *cdd_cartesian_position_motion;
    CDDynamics *cdd_cartesian_orientation_motion;
    CDDynamics *cdd_cartesian_orientationQ_motion;


    MathLib::Vector v_max_intensity;
    MathLib::Vector v_max_intensity_filtered;


    MathLib::Vector finger_enabled;


    Vector fingerReachedLimit;

    int i_contact_level;
    bool b_auto_orientation;


    bool b_use_isotropic_impedance;
    bool b_simulate_tekscan;

    bool b_ravin_jointpos_reach;
    bool b_ravin_cart_reach;
    bool b_ravin_close_mode;
    bool b_ravin_task_mode;
    bool b_ravin_grasped_mode;

    int i_pd_grasp;
    ros::Time time_got_task_signal;

    double d_impedance_lin_stiff;
    double d_impedance_lin_damp;
    double d_impedance_rot_stiff;
    double d_impedance_rot_damp;

    double d_joint_position_kp;
    double d_joint_position_kd;

    Eigen::Matrix3d normal_Frame;

    Eigen::Vector3d contact_normal_filtered;

    TaskRotation *task_rotation;

    //  NewTaskRavin* task_ravin;
    TaskHammering *task_hammering;


    void jacobianSpeXRowPerContactFromContactID(int cid,
                                                MathLib::Matrix &jacobian,
                                                bool X = true,
                                                bool Y = false,
                                                bool Z = false);

    // / Version that uses the link list instead of the contact list.
    void jacobianSpeXRowPerContactFromLinkID(int cid,
                                             MathLib::Matrix &jacobian,
                                             bool X = true,
                                             bool Y = false,
                                             bool Z = false);


    void jacobianSpe1RowPerContactFromLinkID_e(std::vector<TactileContact*> &list_links,
                                               int cid,
                                               Mat &jacobian);

    Mat jacobianSpe1RowPerContactFromLinkID_e(std::vector<TactileContact*> &list_links,
                                              int cid);


    MathLib::Matrix get_full_vector_from_short(MathLib::Matrix shortv,
                                               unsigned int finger);

    Mat get_full_vector_from_short_e(Mat shortv,
                                     unsigned int finger);

    void SendGazeboImpedanceValues(double lin_stiff,
                                   double rot_stiff,
                                   double lin_damp,
                                   double rot_damp,
                                   bool b_reset_position = false);

    void SendGazeboImpedanceValuesFree();


    void NullSpaceThread_c(); // the thread

    void setContactLevel2(int level,
                          std::vector<std::vector<int> > contact_levels);

    // Give a direction and distance of motion, with associated time or velocity
    void SetTraj(double distance,
                 int direction);

    void Compute_JSIM();


    Eigen::Matrix3d FrameFromNormal(Vector3d contact_normal,
                                    int axis = 0);

    Eigen::Matrix3d FrameFromNormal(Vector3d contact_normal,
                                    Vector3d contact_normal2,
                                    int axis = 0,
                                    bool next_axis = 1);


    Vec v_cart_desired_pos_final;
    Vec v_cart_desired_pos_tracked;

    double d_cart_vel_desired;

    double d_joint_damping_coefficient;
    bool b_ravin_task_mode_allowed;

    // signal sent to me by ravin to switch to task mode
    bool b_task_mode_signal;
    bool b_task_rotation;
    bool b_task_hammering;

    // signal sent to me by ravin to start closing
    bool b_close_signal;
    bool b_close_allowed; // the fingers will close because fingers will be in torque mode
    bool b_tighten_signal;


    PIDController pid_controller_cartesian;
    PIDController pid_controller_joint;


    KDL::Frame kdlf_endeff_pos_7_link;
    KDL::Frame kdlf_hand_base_pose;

    KDL::Frame kdlf_hand_debug;


    //  Vec v_cart_endeff_pos;
    Eigen::Vector3d v_cart_endeff_pos;

    //  Mat v_cart_endeff_orient;
    Eigen::Matrix3d v_cart_endeff_orient;

    Vec v_ravin_joint_pos_tracked;
    Vec v_ravin_joint_pos_grasped;

    Vec v_explore_joint_pos_simple;

    bool b_reaching_trajectory;


    unsigned int nbinComp;
    unsigned int nboutComp;

    MathLib::Vector inComp;  // input components
    MathLib::Vector inV;     // input vector
    MathLib::Vector outComp; // output components
    MathLib::Vector outV;
    MathLib::Matrix sigma;   // output covariance vector

    MathLib::Vector regOut[NB_FINGERS];


    // Some stuff from the thread: keep it already allocated:
    MathLib::Matrix m_jsim_tot;


    std::ostringstream ossG;

    std::ostringstream ossGv;    // chain of variable length: to display before the
    std::ostringstream ossDebug; // chain of variable length: to display before the
    std::ostringstream oss_tasks; // chain of variable length: to display before the
    // second one
    std::ostringstream ossT;


    ros::Publisher myPublisher;
    ros::Publisher pub_task_state;

    tf2_ros::TransformBroadcaster *br;

    volatile bool b_new_Jacobian;
    volatile bool b_new_Jacobian_c;
    MathLib::Matrix m_total_NS;
    MathLib::Matrix m_total_NS_nodyn;
    MathLib::Matrix m_total_NS_nodyn2;
    MathLib::Matrix m_full_contact_jacobian;
    /* Vert. concatenation of the
                                                 jacobians */
    MathLib::Matrix m_full_contact_jacobian_noundes;
    /* Vert. concatenation of the
                                                         jacobians */

    bool b_use_virtual_contact_torques;

    bool b_use_joint_centering;    // Add task to center joints
    bool b_avoid_joint_limits;     // Add task to avoid joint limits
    bool b_gazebo_object_imp_zero; // Send 0 impedance values to gazebo for its object impedance control

    double d_ns_torques_multiplier; // multiplier on the PD torques when used in the nullspace

    double d_grav_comp_multiplier;


    Vec v_hand_open_posture;
    Vec v_hand_close_posture;
    Vec v_hand_explo_posture;

    bool b_got_hand_explo_posture;

    void test();

  private:

    Vector3 jointRotAxis;
    int jointRotAxisNum;
    //    TactileContact *mContact_Old;
    std::vector<TactileContact*> list_contacts;
    //    std::vector<TactileContact> list_links;
    std::vector<TactileContact*> list_links;



    //    TactileContact *mVirtualContactList; // in order to apply forces on links not
    // touching, different because projected
    // on the null space of others ..


    std::condition_variable bc_condition_c;
    std::thread bt_thread1_c;
    std::mutex bm_mutex_main_c;


    TaskList tasklist_permanent;
    OperationalSpaceControl ope_space_controller_permanent;

    Vec tot_torques_permanent;


    // New stuff for receiving from topic (CALLBACKS)

  private:

    // Get spacemouse data
    void UpdateSpaceNavDataTwist(const geometry_msgs::Twist::ConstPtr &_msg);

    // Get spaceMouse Button information (and other raw data)
    void UpdateSpaceNavDataJoy(const sensor_msgs::Joy::ConstPtr &_msg);

    // Get Trajectory to reach an object to grasp
    void RavinCallbackTrajectory(const sensor_msgs::JointStateConstPtr &_msg);

    // Get patch inf
    void RavinCallbackMode(const my_msgs::ModeConstPtr &_msg);

    // Get information about the axis of rotation
    void RavinCallbackTaskCartesianRotation(const my_msgs::TaskConstPtr &_msg);

    //  void RavinCallbackTaskHammering(const std_msgs::EmptyConstPtr& _msg);

    // Get "close" message
    void RavinCallbackEmptyClose(const std_msgs::EmptyConstPtr &_msg);

    void RavinCallbackEmptyTighten(const std_msgs::EmptyConstPtr &_msg);

    // Get a signal from the voice recognition node
    void AudioRecognitionCallback(const std_msgs::StringConstPtr &_msg);

    // get a command from a string
    void GetCommandString(const std_msgs::StringConstPtr &_msg);

    void CallbackGraspingPerturbation(const std_msgs::Float32MultiArrayConstPtr &_msg);

    // Get Gazebo robot's joint positions
    void UpdateGazeboJointState(const std_msgs::Float32MultiArrayConstPtr &_msg);

    // Get Allegro's joint positions
    void CallbackAllegroJointState(const sensor_msgs::JointState &_msg);

    // Get Gazebo's contacts
    void UpdateSimContactState(const gazebo_msgs::ContactsState::ConstPtr &_msg);

    // Get Gazebo's contacts
    void RobotTrajectoryCallBack(const moveit_msgs::RobotTrajectoryConstPtr &_msg);

    // Get Gazebo's contacts
    void HandConfigCallback(const std_msgs::Float32MultiArrayConstPtr &_msg);
    void HandStateCallback(const sensor_msgs::JointState &_msg);

    // Publish state if different from current one (to Ravin)
    void PublishState(std::string str);

    std::string last_state_sent;


    sensor_msgs::JointState msg_joint_state_;
    gazebo_msgs::ContactsState msg_contact_state_;
    std::vector<gazebo_msgs::ContactState> stdvec_gazebo_contacts;
    std::map<string, string> stdmap_link_name_from_gazebo;

    std::vector<string> stdvec_link_names_gazebo;

    std::vector<Eigen::Vector3d> stdvec_des_cart_positions;
    int i_curr_despos_i;


    std::vector<joint_limits_interface::JointLimits> stdv_limits; // joint limits


    std::vector<std::vector<int> > contact_levels;

  private:

    void UpdateJacobianAndFk(std::vector<TactileContact*> &contact_list,
                             int i_nb_of_contacts,
                             bool b_debug2 = false,
                             bool b_contact_rotation = false);

  private:

    //  Vector GetShortJointPos(int i_dof_number);


    // / \brief The custom callback queue thread function. (not sure if used ..)

  private:

    void QueueThread();

    // / \brief A pointer to the ROS node.  A node will be instantiated if it does
    // not exist.

    ros::NodeHandle *rosnode_;

    ros::Subscriber sub_ravin_mode;
    ros::Subscriber sub_ravin_close;
    ros::Subscriber sub_ravin_hammer;
    ros::Subscriber sub_ravin_tighten;
    ros::Subscriber sub_ravin_trajectory;
    ros::Subscriber sub_string_command;
    ros::Subscriber sub_grasping_perturbation;
    ros::Subscriber sub_space_nav_joy;
    ros::Subscriber sub_pyconsole;
    ros::Publisher pub_pyconsole;
    ros::Publisher pub_ravin_reached;
    ros::Publisher pub_gazebo_object_impedance;

    //  ros::Publisher  pub_hand_position;
    ros::Publisher pub_hand_state;
    ros::Publisher pub_robot_state;
    ros::Publisher pub_robot_state2;
    ros::Publisher pub_info_ravin;
    ros::Publisher pub_string_debug;

    ros::Subscriber sub_audio_recognition;
    ros::Subscriber sub_moveit_planned_trajectory;
    ros::Subscriber sub_des_hand_config;


    dynamic_reconfigure::Server<my_msgs::my_dyn_paramsConfig> *dyn_reconf_server;
    dynamic_reconfigure::Server<my_msgs::my_dyn_paramsConfig>::CallbackType dyn_reconf_callback_binded;


    ros::Subscriber sub_space_nav_twist;
    ros::Publisher pub_torques;
    ros::Publisher pub_array;
    ros::Publisher pub_pcl;
    ros::Publisher pub_cart_target_gazebo;
    ros::Publisher pub_tactile_pressures;
    ros::Publisher pub_marker_array;
    ros::Publisher pub_hand_state_des_torque;
    ros::Subscriber sub_hand_state;
    ros::Subscriber sub_ravin_task;
    ros::Subscriber sub_joint_state2;
    ros::Subscriber sub_contact_state;

    std::string topic_name_spacenav_twist;          // twist data
    std::string topic_name_spacenav_joy;            // joystick data (for buttons)
    std::string topic_name_ravin_reached;           // twist data
    std::string topic_name_ravin_trajectory;        // twist data
    std::string topic_name_ravin_mode;              // twist data
    std::string topic_name_gazebo_object_impedance; // twist data
    std::string topic_name_hand_pose;               // twist data
    std::string topic_name_audio_recognition;

    std::string topic_name_joint_state2;
    std::string topic_name_hand_joint_state;
    std::string topic_name_contact_state;
    std::string link_name_;


    std::vector<RobotObstacle> r_obstacles;


    // / \brief for setting ROS name space
    std::string robot_namespace_;
    ros::CallbackQueue queue_;


    // / \brief Container for the wrench force that this plugin exerts on the body.
    geometry_msgs::Twist twist_msg_;
    sensor_msgs::Joy joy_msg_;

    std::vector<int> stdvec_prev_joy_buttons;

    double d_lasttime_button0;
    double d_lasttime_button1;

    int i_side_0_major;
    int i_side_1_major;
    bool b_ravins_mode;

    bool b_use_ravins_method_for_vc;

    bool  b_got_trajectory=false;
    double d_time_start_trajectory;
    moveit_msgs::RobotTrajectory planned_trajectory;
    Vec desired_planned_joint_position = Vec::Zero(NB_DOF_TOT);



    void callback(my_msgs::my_dyn_paramsConfig &config,
                  uint32_t level);


    // KDL stuff

  private:

    /* Main tree, loaded at the start */
    KDL::Tree my_tree;


    /* Chains for the kinematic trees of each fingertip */
    KDL::Chain kdlchain_f0;
    KDL::Chain kdlchain_f1;
    KDL::Chain kdlchain_f2;
    KDL::Chain kdlchain_f3;
    KDL::Chain kdlchain_endeff_7_link;
    KDL::Chain kdlchain_hand_base;


    //  KDL::ChainFkSolverPos kdlfksolver_hand_base;
    KDL::ChainFkSolverPos_recursive *kdlfksolver_hand_base;
    KDL::ChainFkSolverPos_recursive *kdlfksolver_debug;

    //  KDL::chain kdlfksolver_hand_base;

    /* Chain solvers for each chain */
    KDL::ChainDynParam *kdlcdp_f0;
    KDL::ChainDynParam *kdlcdp_f1;
    KDL::ChainDynParam *kdlcdp_f2;
    KDL::ChainDynParam *kdlcdp_f3;

    /* JSIM matrix for each fingertip */
    MathLib::Matrix mat_JSIM_f0;
    MathLib::Matrix mat_JSIM_f1;
    MathLib::Matrix mat_JSIM_f2;
    MathLib::Matrix mat_JSIM_f3;

    /* JSIM but in KDL format*/
    KDL::JntSpaceInertiaMatrix jointspace_inertia_f0;
    KDL::JntSpaceInertiaMatrix jointspace_inertia_f1;
    KDL::JntSpaceInertiaMatrix jointspace_inertia_f2;
    KDL::JntSpaceInertiaMatrix jointspace_inertia_f3;

    /* Array of joints for each chain (for each fingertip) */
    KDL::JntArray kdl_jntarray_f0;
    KDL::JntArray kdl_jntarray_f1;
    KDL::JntArray kdl_jntarray_f2;
    KDL::JntArray kdl_jntarray_f3;

    /* Array of joint velocities for each chain (for each fingertip) */
    KDL::JntArray kdl_jntarray_vel_f0;
    KDL::JntArray kdl_jntarray_vel_f1;
    KDL::JntArray kdl_jntarray_vel_f2;
    KDL::JntArray kdl_jntarray_vel_f3;


    /* Array of joint torques from Coriolis forces */
    KDL::JntArray kdl_jntarray_coriolis_f0;
    KDL::JntArray kdl_jntarray_coriolis_f1;
    KDL::JntArray kdl_jntarray_coriolis_f2;
    KDL::JntArray kdl_jntarray_coriolis_f3;

    MathLib::Vector v_torques_coriolis;
    void RespondToConsoleCommandByTopic(const std_msgs::StringConstPtr &_msg);

    RobotInterface::Status RobotUpdateRun();

    template<typename Type_param>
    void getParamSafeHandle(string param_name, Type_param &value, bool b_display = false) {
      if (this->rosnode_->getParam(param_name, value)) {
        if (b_display) cout << param_name + ":\t" << value << endl;
      } else {
        cerr << param_name + " is missing from the parameter server" << endl;
        exit(0);
      }
    }

};


// A function to get values from the parameter server
//template<typename Type_param>
//void getParamSafe(string param_name, Type_param &value, bool b_display = false) {
//    if (ros::param::get(param_name, value)) {
//        if (b_display) cout << param_name + ":\t" << value << endl;
//    } else {
//        cerr << param_name + " is missing from the parameter server" << endl;
//        exit(0);
//    }
//}
#endif // ifndef RobotHapticController_H_
