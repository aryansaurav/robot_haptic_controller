
#include "robot_haptic_controller_helper.h"
#include "moveit/robot_trajectory/robot_trajectory.h"

// #define NB_DOF_HAND 16
// #define NB_DOF_ARM 7
// #define NB_DOF_TOT NB_DOF_ARM + NB_DOF_HAND
// #define NB_FINGERS 4
// #define NB_DOF_FINGER 4


void build_virtual_contact_list(std::vector<TactileContact *>& list_links,
                                MathLib::Vector              & v_contact_on_link_allowed,
                                MathLib::Vector              & finger_enabled,
                                CtrlMode                      *jointCtrlMode,
                                 bool *b_fingers_should_open) {
    for (int i = NB_DOF_ARM; i < NB_DOF_TOT; i++) { // Do that only for the hand (ok)
        int i_finger       = (i - NB_DOF_ARM) / 4;
        int i_finger_joint = (i - NB_DOF_ARM) % 4;

        if ( v_contact_on_link_allowed(i) ) {                   // Link is allowed to have a virtual contact
            if (finger_enabled(i_finger) == 1) {                // Finger is active
                if(!b_fingers_should_open[i_finger]){
                if (list_links[i]->contact_status != CONTACT) { // No contact on this link
                    if ( (jointCtrlMode[i] == TORQUE) || 1 ) {  // Torque mode (not in PID/PID static mode) /// TO TESTS
                        // / Position/normal on the link depends on the joint #
                        MathLib::Vector3 v3_position_in_link_temp;
                        MathLib::Vector3 v3_normal_in_link_temp;
                        bool b_is_valid_link = true;

                        // For other fingers than the thumb
                        if (i_finger != 3) {
                            switch (i_finger_joint) {
                            case 0:

                                // We need this case as a hack for ravin's experiment
                                b_is_valid_link = false;
                                break;

                            case 1:

                                //                v3_position_in_link_temp.Set(0.0, -0.02, 0.00);
                                v3_position_in_link_temp.Set(0.0, -0.025, 0.00);
                                v3_normal_in_link_temp.Set(1.0, 0.0, 0.0); // normal is x
                                break;

                            case 2:
                                v3_position_in_link_temp.Set(0.02, 0.0, 0.0);
                                v3_normal_in_link_temp.Set(0.0, 1.0, 0.0); // normal is y
                                break;

                            case 3:
                                v3_position_in_link_temp.Set(0.02, 0.0, 0.00);
                                v3_normal_in_link_temp.Set(0.0, 1.0, 0.0); // normal is y
                                break;
                            }
                        }
                        else {
                            // Thumb
                            switch (i_finger_joint) {
                            case 0:

                                // We need this case as a hack for ravin's experiment
                                b_is_valid_link = false;
                                break;

                            case 1:
                                b_is_valid_link = false;
                                break;

                            case 2:

                                //                v3_position_in_link_temp.Set(0.0, -0.02, 0.0);
                                //                v3_position_in_link_temp.Set(0.0, -0.03, 0.0);
                                v3_position_in_link_temp.Set(-0.005, -0.03, 0.0);
                                v3_normal_in_link_temp.Set(1.0, 0.0, 0.0); // normal is x
                                break;

                            case 3:

                                //                v3_position_in_link_temp.Set(0.03, 0.0, 0.00);
                                v3_position_in_link_temp.Set(0.03, -0.005, 0.00);

                                //                v3_position_in_link_temp.Set(0.02, 0.0, 0.00);
                                v3_normal_in_link_temp.Set(0.0, 1.0, 0.0); // normal is y
                                break;
                            }
                        }

                        // / mLinkList Version
                        list_links[i]->setPositionInLinkFrame(v3_position_in_link_temp);
                        list_links[i]->setNormalInLinkFrame(v3_normal_in_link_temp); // This one is NOT useful
                        list_links[i]->i_dof_n = i;

                        //            list_links[i]->i_start_dof_n    = NB_DOF_ARM + i_finger * 4;

                        // only if it's in torque mode, really make it a virtual contact
                        if ( (jointCtrlMode[i] == TORQUE) && b_is_valid_link ) {
                            // / If it's newly a virtual contact (status not yet 2), put desired position to 0 as a signal for reset
                            // / This is not working anymore because points now start being virtual contacts. Itś not really their status
                            // / 2 ways:
                            // / - change here, I reset depending on another factor
                            // / - change back so that a linkś status is true, but it still can be computed
                            // / - add a status "non-activated VC" ?
                            if (list_links[i]->contact_status != VIRTUAL_CONTACT) {
                                list_links[i]->v_desired_cart_position.setZero();
                            }

                            list_links[i]->contact_status    = VIRTUAL_CONTACT; // Define status as virtual contact
                            list_links[i]->i_contact_counter = 20;              // should be CONTACT_COUNTER_RESET if 20, it's a new contact, if reached 0, no contact anymore
                        }
                    }
                }
            }
        }
    }
    }
}

void define_control_mode(CtrlMode *jointCtrlMode,
                         Vector  & v_contact_on_link_allowed,
                         Vector  & v_finger_contact_max_dof,
                         Vector  & finger_enabled,
                         int       i_nb_contacts,
                         bool      b_ravin_close_mode,
                         bool      b_close_allowed)
{
    // Reset to torque By default
    for (int i = 0; i < NB_DOF_TOT; i++) {
        jointCtrlMode[i] = TORQUE;
    }


    // If a finger is enabled, put relevant joints in position mode, or in pid static, or in torque mode.
    for (int finger = 0; finger < NB_FINGERS; finger++) {
        bool b_finger_contact             = true;
        int  offset_finger                = finger * NB_DOF_FINGER + NB_DOF_ARM;
        bool b_finger_has_desired_contact = (v_contact_on_link_allowed.GetSubVector(offset_finger, NB_DOF_FINGER).Sum() > 0);

        if (v_finger_contact_max_dof[finger] == -1) {
            b_finger_contact = false;
        }


        if (finger_enabled[finger]) {
            for (int i_finger_joint = 0; i_finger_joint < 4; i_finger_joint++) {
                // Either after a contact on a finger, or on a finger without contact and without any allowed contact link
                if ( ( (i_nb_contacts > 0) || b_ravin_close_mode || b_close_allowed )   &&   (b_finger_contact || b_finger_has_desired_contact) ) {
                    jointCtrlMode[i_finger_joint + offset_finger] = TORQUE;
                } else {
                    jointCtrlMode[i_finger_joint + offset_finger] = POSITION_PID;
                }
            }
        }
    }
}

void define_control_mode_emg(CtrlMode *jointCtrlMode,
                         Vector  & v_contact_on_link_allowed,
                         Vector  & v_finger_contact_max_dof,
                         Vector  & finger_enabled,
                         bool *b_fingers_should_open
                        )
{
    // Reset to torque By default
    for (int i = 0; i < NB_DOF_TOT; i++) {
        jointCtrlMode[i] = TORQUE;
    }


    // If a finger is enabled, put relevant joints in position mode, or in pid static, or in torque mode.
    for (int finger = 0; finger < NB_FINGERS; finger++) {
        bool b_finger_contact             = true;
        int  offset_finger                = finger * NB_DOF_FINGER + NB_DOF_ARM;
        bool b_finger_has_desired_contact = (v_contact_on_link_allowed.GetSubVector(offset_finger, NB_DOF_FINGER).Sum() > 0);

        if (v_finger_contact_max_dof[finger] == -1) {
            b_finger_contact = false;
        }


        if (finger_enabled[finger]) {
            for (int i_finger_joint = 0; i_finger_joint < 4; i_finger_joint++) {
                // Either after a contact on a finger, or on a finger without contact and without any allowed contact link
                // Either if there is another contact on a finger, but not if there is a flag to cancel it ...
//                bool b_finger_should_open = false;
//                if(b_fingers_should_open[finger*4+1] + b_fingers_should_open[finger*4+2] < 25.0 /180.0 * M_PI){
//                    b_finger_should_open = true;
//                }


                if((b_finger_contact && b_finger_has_desired_contact) && (!b_fingers_should_open[finger])){ // TODO add second condition if necessary
//                if ( ( (i_nb_contacts > 0) || b_ravin_close_mode || b_close_allowed )   &&   (b_finger_contact || b_finger_has_desired_contact) ) {
                    jointCtrlMode[i_finger_joint + offset_finger] = TORQUE;
                } else {
                    jointCtrlMode[i_finger_joint + offset_finger] = POSITION_PID;
                }
            }
        }
    }
}



void ComputeFingerOpenCriteria(Vector v_desired_hand_pos_filtered, double d_angle_sum_max_limit, bool *b_fingers_should_open)
{
    for (int finger = 0; finger < NB_FINGERS; finger++) {
        b_fingers_should_open[finger] = false;
        if((v_desired_hand_pos_filtered[finger*4+1] + v_desired_hand_pos_filtered[finger*4+2]) < d_angle_sum_max_limit /180.0 * M_PI){
            b_fingers_should_open[finger] = true;
        }
        
        /*
        if(finger ==2){
            if(v_desired_hand_pos_filtered[finger*4+1] + v_desired_hand_pos_filtered[finger*4+2] < d_angle_sum_max_limit/2.0 /180.0 * M_PI){
                b_fingers_should_open[finger] = true;
            }
        }
        */
    }
}


Eigen::Vector3d compute_main_normal(std::vector<TactileContact *>& list_contacts,
                                    int                            i_nb_contacts,
                                    int                          & i_nb_allowed_contacts_status,
                                    bool                           b_use_mid) {
    Eigen::Vector3d contact_normal, mid_contact_normal, av_contact_normal;

    contact_normal.setZero();
    av_contact_normal.setZero();
    mid_contact_normal.setZero();

    double max_angle = 0;
    int    c1, c2 = -1;


    for (int i = 0; i < i_nb_contacts; i++) {
        // Only in on the hand
        if (list_contacts[i]->i_dof_n > NB_DOF_ARM) {
            // Average normal of contact contact
            Vec n1 = M2E_v( list_contacts[i]->normalInBaseFrame() );

            av_contact_normal += n1; // TODO

            if (i_nb_allowed_contacts_status == 0) {
                i_nb_allowed_contacts_status = 1;
            }

            // find two most different contacts
            for (int j = i + 1; j < i_nb_contacts; j++) {
                if (list_contacts[i]->i_dof_n > NB_DOF_ARM) {
                    Vec n2           = M2E_v( list_contacts[j]->normalInBaseFrame() );
                    double new_angle = acos( n1.dot(n2) ) * 180.0 / PI;

                    if (max_angle < new_angle) {
                        max_angle                    = new_angle;
                        c1                           = i;
                        c2                           = j;
                        i_nb_allowed_contacts_status = 2;
                    }
                }
            }
        }
    }

    // Get mid_contact_normal from the two most distant
    if (i_nb_allowed_contacts_status > 1) { // 2 normals have been found
        Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> > m_link_orientation_c1(list_contacts[c1]->_kdl_frame.M.data);
        Eigen::Map<Eigen::Matrix<double, 3, 3, RowMajor> > m_link_orientation_c2(list_contacts[c2]->_kdl_frame.M.data);

        mid_contact_normal = ( m_link_orientation_c1.col(2) + m_link_orientation_c2.col(2) ) / 2.0;
    } else { // otherwise, copy the average contact normal
        mid_contact_normal = av_contact_normal;
    }


    // / Choose which normal to use
    if (b_use_mid == true) {
        contact_normal = mid_contact_normal;
    } else {
        contact_normal = av_contact_normal;
    }

    // / Normalize the normal
    if (contact_normal.norm() < 0.0001) {
        contact_normal(0) = 1.0; // If vector is null, force it to something before normalization
    }

    contact_normal.normalize();

    return contact_normal;
}

Eigen::Vector3d compute_main_normal_links(std::vector<TactileContact *>& list_links,
                                          int                          & i_nb_allowed_contacts_status,
                                          bool                           b_use_mid) {
    Eigen::Vector3d contact_normal, mid_contact_normal, av_contact_normal;

    contact_normal.setZero();
    av_contact_normal.setZero();
    mid_contact_normal.setZero();

    double max_angle = 0;
    int    c1, c2 = -1;


    for (int i = 0; i < NB_DOF_TOT; i++) {
        if ( (list_links[i]->contact_status == CONTACT) ) {
            // Only in on the hand
            if (list_links[i]->i_dof_n > NB_DOF_ARM) {
                // Average normal of contact contact
                Vec n1 = M2E_v( list_links[i]->normalInBaseFrame() );

                av_contact_normal += n1; // TODO

                if (i_nb_allowed_contacts_status == 0) {
                    i_nb_allowed_contacts_status = 1;
                }

                // find the two most different contacts
                for (int j = i + 1; j < NB_DOF_TOT; j++) {
                    if ( (list_links[j]->contact_status == CONTACT) ) {
                        if (list_links[j]->i_dof_n > NB_DOF_ARM) {
                            Vec n2           = M2E_v( list_links[j]->normalInBaseFrame() );
                            double new_angle = acos( n1.dot(n2) ) * 180.0 / PI;

                            if (max_angle < new_angle) {
                                max_angle                    = new_angle;
                                c1                           = i;
                                c2                           = j;
                                i_nb_allowed_contacts_status = 2;
                            }
                        }
                    }
                }
            }
        }
    }

    // Get mid_contact_normal from the two most distant
    if (i_nb_allowed_contacts_status > 1) { // 2 normals have been found
        Vec n1 = M2E_v( list_links[c1]->normalInBaseFrame() );
        Vec n2 = M2E_v( list_links[c2]->normalInBaseFrame() );
        mid_contact_normal = (n1 + n2) / 2.0;
    } else { // otherwise, copy the average contact normal
        mid_contact_normal = av_contact_normal;
    }

    // / Choose which normal to use
    if (b_use_mid == true) {
        contact_normal = mid_contact_normal;
    } else {
        contact_normal = av_contact_normal;
    }

    // / Normalize the normal
    if (contact_normal.norm() < 0.0001) {
        contact_normal(0) = 1.0; // If vector is null, force it to something before normalization
    }

    contact_normal.normalize();

    return contact_normal;
}

void process_tactile_contact_gazebo(MathLib::Vector& v_finger_contact_max_dof,
                                    int& i_nb_contacts,
                                    std::vector<gazebo_msgs::ContactState>& stdvec_gazebo_contacts,
                                    std::map<string, string>& stdmap_link_name_from_gazebo,
                                    Robot *mRobot,
                                    std::vector<TactileContact *>& list_contacts,
                                    std::vector<TactileContact *>& list_links,
                                    double d_desired_normal_pressure) {
    // Reset the pressures:
    for (int i = 0; i < NB_DOF_TOT; i++) list_links[i]->d_pressure = 0.0;

    // rest vector of furthest contact per finger
    v_finger_contact_max_dof.One();
    v_finger_contact_max_dof.SMinus();

    i_nb_contacts = 0;


    for (int i = 0; i < stdvec_gazebo_contacts.size(); i++) {
        Vector3 v3_pressure;
        string  s_local_link_name;
        v3_pressure.Set(stdvec_gazebo_contacts[i].total_wrench.force.x, stdvec_gazebo_contacts[i].total_wrench.force.y, stdvec_gazebo_contacts[i].total_wrench.force.z);

        // / Get link number/name
        if ( stdmap_link_name_from_gazebo.find(stdvec_gazebo_contacts[i].collision1_name) != stdmap_link_name_from_gazebo.end() ) {
            s_local_link_name = stdmap_link_name_from_gazebo[stdvec_gazebo_contacts[i].collision1_name];
        } else {
            s_local_link_name = "";
            cerr << "link name not found: " << stdvec_gazebo_contacts[i].collision1_name << endl;
            ROS_FATAL("Link name not found");

            exit(0);
        }

        unsigned int i_link_n_temp = mRobot->GetDOFIndex(s_local_link_name); // a bit less ugly solution

        if (i_link_n_temp < NB_DOF_ARM) {} else {
            list_contacts[i_nb_contacts]->i_dof_n          = mRobot->GetDOFIndex(s_local_link_name);
            list_contacts[i_nb_contacts]->d_pressure       = v3_pressure.Norm();
            list_contacts[i_nb_contacts]->s_link_name_urdf = stdvec_gazebo_contacts[i].collision1_name; // get urdf link name
            list_contacts[i_nb_contacts]->setPositionInLinkFrame(stdvec_gazebo_contacts[i].contact_positions[0].x, stdvec_gazebo_contacts[i].contact_positions[0].y, stdvec_gazebo_contacts[i].contact_positions[0].z);
            list_contacts[i_nb_contacts]->setNormalInLinkFrame(stdvec_gazebo_contacts[i].contact_normals[0].x, stdvec_gazebo_contacts[i].contact_normals[0].y, stdvec_gazebo_contacts[i].contact_normals[0].z);
            list_contacts[i_nb_contacts]->contact_status = CONTACT;


            // / Soooo ugly: copy the values 1by1 in Exactly the same way ...
            list_links[i_link_n_temp]->i_dof_n          = mRobot->GetDOFIndex(s_local_link_name);
            list_links[i_link_n_temp]->d_pressure       = v3_pressure.Norm();
            list_links[i_link_n_temp]->s_link_name_urdf = stdvec_gazebo_contacts[i].collision1_name; // get urdf link name
            list_links[i_link_n_temp]->setPositionInLinkFrame(stdvec_gazebo_contacts[i].contact_positions[0].x, stdvec_gazebo_contacts[i].contact_positions[0].y, stdvec_gazebo_contacts[i].contact_positions[0].z);
            list_links[i_link_n_temp]->setNormalInLinkFrame(stdvec_gazebo_contacts[i].contact_normals[0].x, stdvec_gazebo_contacts[i].contact_normals[0].y, stdvec_gazebo_contacts[i].contact_normals[0].z);
            list_links[i_link_n_temp]->contact_status = CONTACT;

            list_links[i_link_n_temp]->i_contact_counter  = CONTACT_COUNTER_RESET;     // if 20, it's a new contact, if reached 0, no contact anymore ...
            list_links[i_link_n_temp]->i_finger_n         = floor( (i_link_n_temp - NB_DOF_ARM) / 4 );
            list_links[i_link_n_temp]->d_desired_pressure = d_desired_normal_pressure; // global value, can be changed afterwards


            if (i_link_n_temp > NB_DOF_ARM - 1) {                                      // only if contact on a finger, not the arm
                int finger = (i_link_n_temp - NB_DOF_ARM) / 4;                         // finger of this contact

                if (v_finger_contact_max_dof[finger] < (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4) {
                    v_finger_contact_max_dof[finger] = (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4;
                }
            }

            i_nb_contacts++;
        }
    }
}

// ///////////////////////////////////
// / \brief process_tactile_contact_gazebo2: (fix) Take into account contact below the hand
// / \param v_finger_contact_max_dof
// / \param i_nb_contacts
// / \param stdvec_gazebo_contacts
// / \param stdmap_link_name_from_gazebo
// / \param mRobot
// / \param list_contacts
// / \param list_links
// / \param d_desired_normal_pressure
// /

void process_tactile_contact_gazebo2(MathLib::Vector& v_finger_contact_max_dof,
                                     int& i_nb_contacts,
                                     std::vector<gazebo_msgs::ContactState>& stdvec_gazebo_contacts,
                                     std::map<string, string>& stdmap_link_name_from_gazebo,
                                     Robot *mRobot,
                                     std::vector<TactileContact *>& list_contacts,
                                     std::vector<TactileContact *>& list_links,
                                     double d_desired_normal_pressure) {
    // Reset the pressures:
    for (int i = 0; i < list_links.size(); i++) list_links[i]->d_pressure = 0.0;

    // rest vector of furthest contact per finger
    v_finger_contact_max_dof.One();
    v_finger_contact_max_dof.SMinus();

    i_nb_contacts = 0;


    for (int i = 0; i < stdvec_gazebo_contacts.size(); i++) {
        Eigen::Vector3d v3e_pressure;
        tf::vectorMsgToEigen(stdvec_gazebo_contacts[i].total_wrench.force, v3e_pressure);
        string s_local_link_name;


        // / Get link number/name
        if ( stdmap_link_name_from_gazebo.find(stdvec_gazebo_contacts[i].collision1_name) != stdmap_link_name_from_gazebo.end() ) {
            s_local_link_name = stdmap_link_name_from_gazebo[stdvec_gazebo_contacts[i].collision1_name];
        } else {
            s_local_link_name = "";
            cerr << "link name not found: " << stdvec_gazebo_contacts[i].collision1_name << endl;
            ROS_FATAL("Link name not found");
            continue;
//            exit(0);
        }

        unsigned int i_link_n_temp = mRobot->GetDOFIndex(s_local_link_name); // a bit less ugly solution

        if ( (i_link_n_temp < NB_DOF_ARM) && 0 ) {                           // DEBUG: disable this
            // This is just copied from the next scope
            //        list_links[i_link_n_temp]->i_dof_n          = mRobot->GetDOFIndex(s_local_link_name);
            //        list_links[i_link_n_temp]->d_pressure       = v3e_pressure.norm();
            //        list_links[i_link_n_temp]->s_link_name_urdf = stdvec_gazebo_contacts[i].collision1_name; // get urdf link name
            //        list_links[i_link_n_temp]->setPositionInLinkFrame(stdvec_gazebo_contacts[i].contact_positions[0].x, stdvec_gazebo_contacts[i].contact_positions[0].y, stdvec_gazebo_contacts[i].contact_positions[0].z);
            //        list_links[i_link_n_temp]->setNormalInLinkFrame(stdvec_gazebo_contacts[i].contact_normals[0].x, stdvec_gazebo_contacts[i].contact_normals[0].y, stdvec_gazebo_contacts[i].contact_normals[0].z);
            //        list_links[i_link_n_temp]->contact_status = CONTACT;

            //        list_links[i_link_n_temp]->i_contact_counter  = CONTACT_COUNTER_RESET;     // if 20, it's a new contact, if reached 0, no contact anymore ...
            //        list_links[i_link_n_temp]->i_finger_n         = floor( (i_link_n_temp - NB_DOF_ARM) / 4 );
            //        list_links[i_link_n_temp]->d_desired_pressure = d_desired_normal_pressure; // global value, can be changed afterwards
        } else {
            //        list_contacts[i_nb_contacts] = new  TactileContact();

            list_contacts[i_nb_contacts]->i_dof_n          = mRobot->GetDOFIndex(s_local_link_name);
            list_contacts[i_nb_contacts]->d_pressure       = v3e_pressure.norm();
            list_contacts[i_nb_contacts]->s_link_name_urdf = stdvec_gazebo_contacts[i].collision1_name; // get urdf link name
            list_contacts[i_nb_contacts]->setPositionInLinkFrame(stdvec_gazebo_contacts[i].contact_positions[0].x, stdvec_gazebo_contacts[i].contact_positions[0].y, stdvec_gazebo_contacts[i].contact_positions[0].z);
            list_contacts[i_nb_contacts]->setNormalInLinkFrame(stdvec_gazebo_contacts[i].contact_normals[0].x, stdvec_gazebo_contacts[i].contact_normals[0].y, stdvec_gazebo_contacts[i].contact_normals[0].z);
            list_contacts[i_nb_contacts]->contact_status = CONTACT;


            // / Soooo ugly: copy the values 1by1 in Exactly the same way ...
            list_links[i_link_n_temp]->i_dof_n          = mRobot->GetDOFIndex(s_local_link_name);
            list_links[i_link_n_temp]->d_pressure       = v3e_pressure.norm();
            list_links[i_link_n_temp]->s_link_name_urdf = stdvec_gazebo_contacts[i].collision1_name; // get urdf link name
            list_links[i_link_n_temp]->setPositionInLinkFrame(stdvec_gazebo_contacts[i].contact_positions[0].x, stdvec_gazebo_contacts[i].contact_positions[0].y, stdvec_gazebo_contacts[i].contact_positions[0].z);
            list_links[i_link_n_temp]->setNormalInLinkFrame(stdvec_gazebo_contacts[i].contact_normals[0].x, stdvec_gazebo_contacts[i].contact_normals[0].y, stdvec_gazebo_contacts[i].contact_normals[0].z);
            list_links[i_link_n_temp]->contact_status = CONTACT;

            list_links[i_link_n_temp]->i_contact_counter = CONTACT_COUNTER_RESET; // if 20, it's a new contact, if reached 0, no contact anymore ...

            if (i_link_n_temp >= NB_DOF_ARM) list_links[i_link_n_temp]->i_finger_n = floor( (i_link_n_temp - NB_DOF_ARM) / 4 );
            else list_links[i_link_n_temp]->i_finger_n = -1;

            list_links[i_link_n_temp]->d_desired_pressure = d_desired_normal_pressure; // global value, can be changed afterwards
            //            cerr << "i_link_n_temp: " << i_link_n_temp << endl;
            //            cerr << "list_links[i_link_n_temp]->i_finger_n: " << list_links[i_link_n_temp]->i_finger_n << endl;


            if (i_link_n_temp > NB_DOF_ARM - 1) {              // only if contact on a finger, not the arm
                int finger = (i_link_n_temp - NB_DOF_ARM) / 4; // finger of this contact

                if (v_finger_contact_max_dof[finger] < (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4) {
                    v_finger_contact_max_dof[finger] = (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4;
                }
            }

            i_nb_contacts++;
        }
    }
}

// ///////////////////////////////////////////
// / \brief process_tactile_contact_tekscan
// / \param v_finger_contact_max_dof
// / \param i_nb_contacts
// / \param full_patch_list
// / \param stdvec_link_names_gazebo
// / \param mRobot
// / \param list_contacts
// / \param list_links
// / \param d_desired_normal_pressure
// /

void process_tactile_contact_tekscan(Vector                       & v_finger_contact_max_dof,
                                     int                          & i_nb_contacts,
                                     std::vector<TekPatch *>      & full_patch_list,
                                     std::vector<string>          & stdvec_link_names_gazebo,
                                     Robot                         *mRobot,
                                     std::vector<TactileContact *>& list_contacts,
                                     std::vector<TactileContact *>& list_links,
                                     double                         d_desired_normal_pressure)
{
    // Reset the pressures:
    for (int i = 0; i < NB_DOF_TOT; i++) list_links[i]->d_pressure = 0.0;

    // reset some values
    v_finger_contact_max_dof.One();
    v_finger_contact_max_dof.SMinus();


    // Process data
    i_nb_contacts = 0;

    for (int i = 0; i < full_patch_list.size(); i++) {
        if ( (full_patch_list[i]->mContactCounter > 0) && full_patch_list[i]->mUsed ) { // also test that the patch is used ..
            bool aloneOnJoint = 1;

            // First, check if another patch is active on the same joint...
            for (int j = 0; j < full_patch_list.size(); j++) {
                if ( (full_patch_list[j]->mContactCounter > 0) && full_patch_list[j]->mUsed && (j != i) )
                    if (full_patch_list[i]->mLinkAttachedName.compare(full_patch_list[j]->mLinkAttachedName) == 0) {
                        //                ossGv << "two on the same patch ... !" << endl;

                        // Keep the one with highest value (not optimal, but keep for now)
                        if (full_patch_list[j]->mAveragePressure > full_patch_list[i]->mAveragePressure) {
                            aloneOnJoint = 0;
                        } else {
                            aloneOnJoint = 1;
                        }
                    }
            }

            if (aloneOnJoint) {
                //                if (i_counter > 215) {                                                                     // wait a little at the beginning
                unsigned int i_link_n_temp = mRobot->GetDOFIndex(full_patch_list[i]->mLinkAttachedName); // a bit less ugly solution
                //        list_contacts[i_nb_contacts] = new  TactileContact();
                list_contacts[i_nb_contacts]->i_dof_n = mRobot->GetDOFIndex(full_patch_list[i]->mLinkAttachedName);

                //              list_contacts[i_nb_contacts]->setLinkDof(full_patch_list[i]->mLinkAttachedName, mRobot);
                list_contacts[i_nb_contacts]->d_pressure       = full_patch_list[i]->mAveragePressure;
                list_contacts[i_nb_contacts]->s_link_name_urdf = stdvec_link_names_gazebo[i_link_n_temp];
                list_contacts[i_nb_contacts]->setPositionInLinkFrame( full_patch_list[i]->mT_Link_2_AvTaxel.GetTranslation() );
                list_contacts[i_nb_contacts]->setNormalInLinkFrame( full_patch_list[i]->mT_Link_2_AvTaxel.GetOrientation().GetColumn(2) ); // mmh, not the best. But it should work
                list_contacts[i_nb_contacts]->contact_status = CONTACT;


                // / Soooo ugly: copy the values 1by1 in Exactly the same way ...
                list_links[i_link_n_temp]->i_dof_n = mRobot->GetDOFIndex(full_patch_list[i]->mLinkAttachedName);

                //              list_links[i_link_n_temp]->setLinkDof(full_patch_list[i]->mLinkAttachedName, mRobot);
                list_links[i_link_n_temp]->d_pressure       = full_patch_list[i]->mAveragePressure;
                list_links[i_link_n_temp]->s_link_name_urdf = stdvec_link_names_gazebo[i_link_n_temp];
                list_links[i_link_n_temp]->setPositionInLinkFrame( full_patch_list[i]->mT_Link_2_AvTaxel.GetTranslation() );
                list_links[i_link_n_temp]->setNormalInLinkFrame( full_patch_list[i]->mT_Link_2_AvTaxel.GetOrientation().GetColumn(2) ); // mmh, not the best. But it should work
                list_links[i_link_n_temp]->contact_status = CONTACT;

                list_links[i_link_n_temp]->d_desired_pressure = d_desired_normal_pressure;                                              // global value, can be changed afterwards


                list_links[i_link_n_temp]->i_finger_n        = floor( (i_link_n_temp - NB_DOF_ARM) / 4 );
                list_links[i_link_n_temp]->i_contact_counter = CONTACT_COUNTER_RESET;

                int finger = (i_link_n_temp - NB_DOF_ARM) / 4;                                                          // finger of this contact

                if (v_finger_contact_max_dof[finger] < (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4) { // compare maxdof of this finger with
                    v_finger_contact_max_dof[finger] = (int)(list_contacts[i_nb_contacts]->i_dof_n - NB_DOF_ARM) % 4;
                }

                i_nb_contacts++;

                //                }
            }
        }
    }
}

// /////////////////////////////////


void distanceToJointLimits(Vec                                             v_curr_position,
                           std::vector<joint_limits_interface::JointLimits>stdv_limits,
                           Vec                                           & ev_dist_2_limits,
                           Vec                                           & ev_dist_2_center,
                           Vec                                           & ev_dist_2_center_percent) {
    if (stdv_limits.size() >= NB_DOF_TOT) {
        for (int i = 0; i < NB_DOF_TOT; i++) {
            double range_joint         = (stdv_limits[i].max_position - stdv_limits[i].min_position);
            double distCenter          = v_curr_position[i] -  (stdv_limits[i].max_position - range_joint / 2.0);
            double dist_center_percent = distCenter / range_joint * 100.0;

            ev_dist_2_center[i]         = distCenter;
            ev_dist_2_center_percent[i] = dist_center_percent; // wrong ???

            double distLow = v_curr_position[i] - stdv_limits[i].min_position;
            double distUp  = stdv_limits[i].max_position - v_curr_position[i];

            if (distLow < distUp) ev_dist_2_limits[i] = distLow * 180. / M_PI;
            else ev_dist_2_limits[i] = -distUp * 180. / M_PI;
        }
    } else {
        ROS_ERROR("Pb with size of joint limits");
    }
}

// /////////////////////////////////////////

// Print this as dashes ....
string getJointLimitsString(const Vec v_dist_2_center_percent,
                            int       i_dashes_max = 20) {
    std::ostringstream ossTemp;

    for (int i = 0; i < v_dist_2_center_percent.rows(); i++) {
        int n_dashes = floor(v_dist_2_center_percent[i] + 50) / 100 * i_dashes_max;
        n_dashes = max(min(i_dashes_max, n_dashes), 0);

        ossTemp << "Joint " << i << ": [";

        for (int i = 0; i < n_dashes; i++) ossTemp << "-";

        for (int i = 0; i < i_dashes_max - n_dashes; i++) ossTemp << " ";
        ossTemp << "]" << endl;
    }
    return ossTemp.str();
}

// //////////
// / \brief getLimitAvoidance Map distance to joint limit to a torque for avoiding it
// / \return
// /
double getLimitAvoidanceTorque(double v_dist_2_center_percent,
                               double d_min_jointlimit_distance_percent,
                               double d_max_torque)
{
    double new_coordinate = abs(v_dist_2_center_percent) - (50.0 - d_min_jointlimit_distance_percent);

    new_coordinate = min(new_coordinate, d_min_jointlimit_distance_percent); // bound in case it's outside the limit

    if (new_coordinate > 0) {
        double amplitude = new_coordinate * (d_max_torque / d_min_jointlimit_distance_percent);

        if (v_dist_2_center_percent > 0) amplitude = -amplitude;
        return amplitude;
    } else return 0;
}

////////////////////////////////////////////
Vec GetFirstTrajectoryPoint(moveit_msgs::RobotTrajectory& trajectory,
                            Vec                         & v_target_joint_position)
{
    trajectory_msgs::JointTrajectoryPoint desired_point;

    cerr << "deb2" << endl;

    if (trajectory.joint_trajectory.points.size() > 1) {
        cerr << "There are " <<  trajectory.joint_trajectory.points.size() << " points" << endl;
        desired_point = trajectory.joint_trajectory.points[1];
        cerr << "deb3" << endl;

        v_target_joint_position.resize( desired_point.positions.size() );

        cerr << "deb4" << endl;

        for (int i = 0; i < NB_DOF_ARM; i++) {
            v_target_joint_position[i] = desired_point.positions[i];
        }
        cerr << "deb5" << endl;
    } else {
        v_target_joint_position.setZero();
        ROS_ERROR("Trajectory from the planner does not have enough points (<2)");
    }
    return v_target_joint_position;
}

///////////////////////////////////////////////
Vec GetTrajectoryPointFromTime(moveit_msgs::RobotTrajectory& trajectory_msg,
                               double                        time,
                               double                        time_scale,
                               Vec                         & v_target_joint_position
                               )
{
//    cerr << "deb01" << endl;
    auto traj_points = trajectory_msg.joint_trajectory.points;
    int i_traj_points_size = traj_points.size();
//    cerr << "deb02" << endl;
//    cerr << "time: " << time << endl;

    if (i_traj_points_size > 1)
    {
//        cerr << "deb03" << endl;
        time *= time_scale;


//        cerr << "i_traj_points_size: " << i_traj_points_size << endl;
//        cerr << "v_target_joint_position.rows(): " << v_target_joint_position.rows() << endl;
//        cerr << "traj_points[i_traj_points_size].time_from_start.toSec(): " <<  traj_points[i_traj_points_size-1].time_from_start.toSec() << endl;



        if (time < 0.0) {
            ROS_FATAL("negative time");
        } else if ( time > traj_points[i_traj_points_size-1].time_from_start.toSec() ) {
            // Copy last point, check if after last point
//            cerr << "deb04" << endl;

            for (int i = 0; i < NB_DOF_ARM; i++) {
                cerr << "deb04 counting: " << i << endl;
                v_target_joint_position[i] = traj_points[i_traj_points_size-1].positions[i];
            }
//            cerr << "deb04b" << endl;
        } else {
//            cerr << "deb05" << endl;
            // Interpolate between points
            int index_before = 0;
            double time_before, time_after;
            double d_alpha_after;

            for (int i = i_traj_points_size-1; i >= 0; i--) {
                if (traj_points[i].time_from_start.toSec() < time) {
                    index_before = i; //update, start at last one anyway.
                    break;
                }
            }
//            cerr << "deb05a" << endl;


            time_before   = traj_points[index_before].time_from_start.toSec();
            time_after    = traj_points[index_before + 1].time_from_start.toSec();
            d_alpha_after = (time - time_before) / (time_after - time_before);
//            cerr << "deb05b" << endl;

            Vec jpos_before(NB_DOF_ARM), jpos_after(NB_DOF_ARM);
//            cerr << "deb05c" << endl;
            cerr << "i_traj_points_size: " << i_traj_points_size << endl;
            cerr << "index_before: " << index_before << endl;


//            ROS_ASSERT(index_before<i_traj_points_size);
            for (int i = 0; i < NB_DOF_ARM; i++) {
                // it does crash here
                jpos_after[i]  = traj_points[index_before + 1].positions[i]; // don't do that  if last point, but why not ?
                jpos_before[i] = traj_points[index_before].positions[i];
            }
//            cerr << "deb05d" << endl;

            v_target_joint_position = (1 - d_alpha_after) * jpos_before + d_alpha_after * jpos_after;
//            cerr << "deb06" << endl;
        }
    } else {
        v_target_joint_position.setZero();
        ROS_ERROR("Trajectory from the planner does not have enough points (<2)");
    }
    return v_target_joint_position;
}

