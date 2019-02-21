#ifndef VIRTUAL_CONTACT_HELPERS_H
#define VIRTUAL_CONTACT_HELPERS_H

#endif // VIRTUAL_CONTACT_HELPERS_H


#include "Eigen/Core"
#include "tekPatch.h"
//#include "MathLib.h"
#include "mathlib_eigen_conversions.h"
#include "globals.h"
#include "ros/ros.h"

using namespace Eigen;


typedef Eigen::VectorXd Vec;
typedef Eigen::MatrixXd Mat;



Vector3d getDirectionOfClosingRavin(std::vector<TactileContact*> &list_links, int i_vc, int i_side_1_major, int i_side_0_major, Vector3d v3_dir_p0_to_p1, bool b_parallel_direction);

Vector3d getDirectionOfClosingClosest(std::vector<TactileContact*> &list_links, int i_vc, bool b_use_vc_normalinfo);
Vector3d getDirectionOfClosingClosestContinuous(std::vector<TactileContact*> &list_links, int i_vc, double d_use_vc_normalinfo);

int getClosestContact(std::vector<TactileContact*> &list_links, int i_vc);
