#include "task_ravin.h"


using namespace std;

bool NewTaskRavin::ExecuteTask(Vector3d& v_cart_pos_tracked_ravin, Matrix3d& v_cart_orient_tracked_ravin) {
  double d_max_time = 25.0;

  bool   b_finished_task;
  double d_time_since_signal = (ros::Time::now() - d_time_start).toSec();

  //  cerr << "d_time_since_signal: " << d_time_since_signal << endl;
  // Maximum Time
  if (d_time_since_signal >= d_max_time) {
    // freeze the motion
    d_time_since_signal = d_max_time;
    b_finished_task     = true;
  }

  ComputeCartPose(d_time_since_signal, v_cart_pos_tracked_ravin, v_cart_orient_tracked_ravin);

  return b_finished_task;
}

void TaskHammering::ComputeCartPose(double d_time_since_signal, Vector3d& v_cart_pos_tracked_ravin, Matrix3d& v_cart_orient_tracked_ravin) {
  ravin_hammering_direction = ravin_hammering_direction / ravin_hammering_direction.norm();

  double d_omega              = 2.0 * M_PI / d_time_period;
  double d_position_modulator = d_amplitude * sin(d_time_since_signal * d_omega);

  v_cart_orient_tracked_ravin = lwr_cart_orientation_init_e;
  v_cart_pos_tracked_ravin    = lwr_cart_position_init_e + ravin_hammering_direction * d_position_modulator;
}

void TaskRotation::ComputeCartPose(double d_time_since_signal, Vector3d& v_cart_pos_tracked_ravin, Matrix3d& v_cart_orient_tracked_ravin) {
  d_ravin_task_rotation = d_time_since_signal * lwr_rotation_velocity;

  //  cerr << "d_ravin_task_rotation: " << d_ravin_task_rotation << endl;

  // During the motion, set the desired cartesian pose
  // 0_T_C * Rot * C_T_0 * 0_T_A
  Eigen::AngleAxisd lwr_rotation_amount(d_ravin_task_rotation, lwr_rotation_axis);

  Transform<double, 3, Affine> O_T_A = Translation3d(lwr_cart_position_init_e) * AngleAxisd(lwr_cart_orientation_init_e);
  Transform<double, 3, Affine> t     = Translation3d(lwr_rotation_point) * lwr_rotation_amount * Translation3d(-lwr_rotation_point) * O_T_A;

  // Copy the tracked data
  v_cart_pos_tracked_ravin    = t.translation();
  v_cart_orient_tracked_ravin = t.rotation();
}
