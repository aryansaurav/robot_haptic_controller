#ifndef ROBOT_HAPTIC_CONTROLLER_HELPER_H
# define ROBOT_HAPTIC_CONTROLLER_HELPER_H
#endif // ROBOT_HAPTIC_CONTROLLER_HELPER_H


#include "Eigen/Core"
using namespace Eigen;
#include "tekPatch.h"
#include "MathLib.h"
#include "globals.h"
#include "gazebo_msgs/ContactState.h"
#include "RobotLib.h"

// #include "tf2_kdl/tf2_kdl.h"
#include "eigen_conversions/eigen_kdl.h"
#include "eigen_conversions/eigen_msg.h"

#include "joint_limits_interface/joint_limits.h"

#include "tuple"

#include "mathlib_eigen_conversions.h"
#include "moveit_msgs/RobotTrajectory.h"

typedef Eigen::VectorXd Vec;

// From the current set of contacts and links with allowed contacts, sets the desired contacts to the list_links
void build_virtual_contact_list(std::vector<TactileContact *>& list_links,
                                MathLib::Vector              & v_contact_on_link_allowed,
                                MathLib::Vector              & finger_enabled,
                                CtrlMode                      *jointCtrlMode, bool *b_fingers_should_open);

// From current state of contact and allowed stuff, define the control mode
void define_control_mode(CtrlMode        *jointCtrlMode,
                         MathLib::Vector& v_contact_on_link_allowed,
                         MathLib::Vector& v_finger_contact_max_dof,
                         MathLib::Vector& finger_enabled,
                         int              i_nb_contacts,
                         bool             b_ravin_close_mode,
                         bool             b_close_allowed);

// when active EMG grasp, it's a different setup
void define_control_mode_emg(CtrlMode        *jointCtrlMode,
                         MathLib::Vector& v_contact_on_link_allowed,
                         MathLib::Vector& v_finger_contact_max_dof,
                         MathLib::Vector& finger_enabled, bool* b_fingers_should_open);

void ComputeFingerOpenCriteria(Vector v_desired_hand_pos_filtered, double d_angle_sum_max_limit, bool* b_fingers_should_open);


// Compute the average or median normal of contact
Eigen::Vector3d compute_main_normal(std::vector<TactileContact *>& list_contacts,
                                    int                            i_nb_contacts,
                                    int                          & i_nb_allowed_contacts_status,
                                    bool                           b_use_mid = false);

Eigen::Vector3d compute_main_normal_links(std::vector<TactileContact *>& list_links,
                                          int                          & i_nb_allowed_contacts_status,
                                          bool                           b_use_mid = false);


void process_tactile_contact_gazebo(MathLib::Vector& v_finger_contact_max_dof,
                                    int& i_nb_contacts,
                                    std::vector<gazebo_msgs::ContactState>& stdvec_gazebo_contacts,
                                    std::map<string, string>& stdmap_link_name_from_gazebo,
                                    Robot *mRobot,
                                    std::vector<TactileContact *>& list_contacts,
                                    std::vector<TactileContact *>& list_links,
                                    double d_desired_normal_pressure);

void process_tactile_contact_gazebo2(MathLib::Vector& v_finger_contact_max_dof,
                                     int& i_nb_contacts,
                                     std::vector<gazebo_msgs::ContactState>& stdvec_gazebo_contacts,
                                     std::map<string, string>& stdmap_link_name_from_gazebo,
                                     Robot *mRobot,
                                     std::vector<TactileContact *>& list_contacts,
                                     std::vector<TactileContact *>& list_links,
                                     double d_desired_normal_pressure);


void process_tactile_contact_tekscan(MathLib::Vector              & v_finger_contact_max_dof,
                                     int                          & i_nb_contacts,
                                     std::vector<TekPatch *>      & full_patch_list,
                                     std::vector<string>          & stdvec_link_names_gazebo,
                                     Robot                         *mRobot,
                                     std::vector<TactileContact *>& list_contacts,
                                     std::vector<TactileContact *>& list_links,
                                     double                         d_desired_normal_pressure);

void distanceToJointLimits(Vec                                             v_curr_position,
                           std::vector<joint_limits_interface::JointLimits>stdv_limits,
                           Vec                                           & ev_dist_2_limits,
                           Vec                                           & ev_dist_2_center,
                           Vec                                           & ev_dist_2_center_percent);

string getJointLimitsString(const Vec v_dist_2_center_percent,
                            int       i_dashes_max);

double getLimitAvoidanceTorque(double v_dist_2_center_percent,
                               double d_min_jointlimit_distance_percent,
                               double d_max_torque);


Vec GetFirstTrajectoryPoint(moveit_msgs::RobotTrajectory& trajectory,
                            Vec                         & v_target_joint_position);

Vec GetTrajectoryPointFromTime(moveit_msgs::RobotTrajectory& trajectory_msg,
                               double                        time, double time_scale,
                               Vec                         & v_target_joint_position);
