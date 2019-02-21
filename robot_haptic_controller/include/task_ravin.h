#ifndef TASK_RAVIN_H
#define TASK_RAVIN_H

#include "Eigen/Core"
#include "Eigen/Dense"
using namespace Eigen;
#include <ros/ros.h>
#include <math.h>


// Ongoing work !
class NewTaskRavin {
  public:
//    NewTaskRavin();

    // Generic variables
    ros::Time d_time_start;

    // these need to be filled ...
    Vector3d lwr_cart_position_init_e;
    Matrix3d lwr_cart_orientation_init_e;

  private:

    virtual void ComputeCartPose(double    d_time_since_signal,
                                 Vector3d& v_cart_pos_tracked_ravin,
                                 Matrix3d& v_cart_orient_tracked_ravin){} // depends on the type of task.

  public:

    bool ExecuteTask(Vector3d& v_cart_pos_tracked_ravin,
                     Matrix3d& v_cart_orient_tracked_ravin);
};


class TaskHammering : public NewTaskRavin {
  public:
//    TaskHammering() : NewTaskRavin(){}
    Vector3d ravin_hammering_direction;
    double   d_time_period;
    double   d_amplitude;

  public:

    void ComputeCartPose(double    d_time_since_signal,
                         Vector3d& v_cart_pos_tracked_ravin,
                         Matrix3d& v_cart_orient_tracked_ravin);
};


class TaskRotation : public NewTaskRavin {
  public:
//    TaskRotation() : NewTaskRavin(){}

    Vector3d lwr_rotation_axis;
    Vector3d lwr_rotation_point;
    double   lwr_rotation_velocity;
    double   d_ravin_task_rotation;

  private:

    void ComputeCartPose(double    d_time_since_signal,
                         Vector3d& v_cart_pos_tracked_ravin,
                         Matrix3d& v_cart_orient_tracked_ravin);
};


#endif // TASK_RAVIN_H
