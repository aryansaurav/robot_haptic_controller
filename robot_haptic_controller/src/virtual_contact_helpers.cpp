



#include "virtual_contact_helpers.h"

//////////////////////
/// \brief getDirectionOfClosingRavin
/// \param list_links
/// \param i_vc
/// \param i_side_1_major
/// \param i_side_0_major
/// \param v3_dir_p0_to_p1
/// \param b_parallel_direction
/// \return
///

Vector3d getDirectionOfClosingRavin(std::vector<TactileContact *>& list_links, int i_vc, int i_side_1_major, int i_side_0_major, Vector3d v3_dir_p0_to_p1, bool b_parallel_direction) {
    Vector3d v3_virt_veldir_in_root_frame;

    int i_opposing_contact;

    if (list_links[i_vc]->i_side == 0) i_opposing_contact = i_side_1_major;
    else i_opposing_contact = i_side_0_major;


    if (b_parallel_direction) {
        // The closing direction depends only on the two main patches
        Vector3d v3_dir_p0_to_p1_signed = v3_dir_p0_to_p1;

        if (list_links[i_vc]->i_side == 1) v3_dir_p0_to_p1_signed = -v3_dir_p0_to_p1_signed;
        v3_virt_veldir_in_root_frame = v3_dir_p0_to_p1_signed;
    } else {
        // The closing direction depends on the current patch and the main opposing one
        Vector3d v3_dir_to_closest_contact( (list_links[i_opposing_contact]->_kdl_frame.p - list_links[i_vc]->_kdl_frame.p).data );
        v3_dir_to_closest_contact   /= v3_dir_to_closest_contact.norm();
        v3_virt_veldir_in_root_frame = v3_dir_to_closest_contact;
    }

    return v3_virt_veldir_in_root_frame;
}

////////////////////////////////////////////////////////
/// \brief getDirectionOfClosingClosest
/// \param list_links
/// \param i_vc
/// \param b_use_vc_normalinfo
/// \return
///
Vector3d getDirectionOfClosingClosest(std::vector<TactileContact *>& list_links, int i_vc, bool b_use_vc_normalinfo) {
    Vector3d v3_virt_veldir_in_root_frame;

    // closest contact
    int i_closest_contact = getClosestContact(list_links, i_vc);

    if (b_use_vc_normalinfo) {
        // SOLUTION 1: along the closest contact's normal
        v3_virt_veldir_in_root_frame = M2E_v( list_links[i_closest_contact]->normalInBaseFrame() );
    } else {
        // SOLUTION 2: towards the closest's contact
        Vector3d v3_dir_to_closest_contact( (list_links[i_closest_contact]->_kdl_frame.p - list_links[i_vc]->_kdl_frame.p).data );
        v3_dir_to_closest_contact   /= v3_dir_to_closest_contact.norm();
        v3_virt_veldir_in_root_frame = v3_dir_to_closest_contact;
    }

    return v3_virt_veldir_in_root_frame;
}

/////////////////
/// \brief getDirectionOfClosingClosestContinuous
/// \param list_links
/// \param i_vc
/// \param d_use_vc_normalinfo
/// \return
///

Vector3d getDirectionOfClosingClosestContinuous(std::vector<TactileContact *> &list_links, int i_vc, double d_use_vc_normalinfo)
{
    ROS_ASSERT_MSG(d_use_vc_normalinfo <= 1.0, "d_use_vc_normalinfo is above 1");
    ROS_ASSERT_MSG(d_use_vc_normalinfo >= 0.0, "d_use_vc_normalinfo is below 0");

    Vector3d v3_virt_veldir_in_root_frame;

    // closest contact
    int i_closest_contact = getClosestContact(list_links, i_vc);

    Vector3d v3_contact_normal = M2E_v( list_links[i_closest_contact]->normalInBaseFrame() );
    Vector3d v3_dir_to_closest_contact( (list_links[i_closest_contact]->_kdl_frame.p - list_links[i_vc]->_kdl_frame.p).data );
    v3_dir_to_closest_contact  /= v3_dir_to_closest_contact.norm();

    v3_virt_veldir_in_root_frame = d_use_vc_normalinfo * v3_contact_normal + (1.0 - d_use_vc_normalinfo) * v3_dir_to_closest_contact;

    return v3_virt_veldir_in_root_frame;
}




////////////////////////////////////////////////////////
/// \brief getClosestContact
/// \param list_links
/// \param i_vc
/// \return
///
int getClosestContact(std::vector<TactileContact *>& list_links, int i_vc) {
    double d_min_contact_distance = 10.0; // 10m: too high on purpose
    int    i_closest_contact      = -1;

    for (int i_c = 0; i_c < NB_DOF_TOT; i_c++) {
        if ( (list_links[i_c]->contact_status == CONTACT) ) {
            double d_contact_distance = (list_links[i_vc]->_kdl_frame.p - list_links[i_c]->_kdl_frame.p).Norm();

            if (d_contact_distance < d_min_contact_distance) {
                d_min_contact_distance = d_contact_distance;
                i_closest_contact      = i_c;
            }
        }
    }

    return i_closest_contact;
}















