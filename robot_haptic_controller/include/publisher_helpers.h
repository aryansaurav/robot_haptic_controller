#ifndef PUBLISHER_HELPERS_H
# define PUBLISHER_HELPERS_H

#endif // PUBLISHER_HELPERS_H

#include "visualization_msgs/MarkerArray.h"
#include "Eigen/Core"
#include "tekPatch.h"
#include "kdl_conversions/kdl_msg.h"

using namespace Eigen;

visualization_msgs::MarkerArray make_contact_marker_array2(Eigen::VectorXd v_contact_on_link_allowed,
                                                 std::vector<TactileContact *> &list_links,
                                                 string s_base_frame);
