


#include "publisher_helpers.h"


visualization_msgs::MarkerArray make_contact_marker_array2(Eigen::VectorXd v_contact_on_link_allowed, std::vector<TactileContact*> &list_links, std::string s_base_frame) {
  visualization_msgs::MarkerArray marker_array;

  for (int i = 0; i < v_contact_on_link_allowed.rows(); i++) {
    visualization_msgs::Marker marker;


    // This depends which rviz is being used: full robot or only hand ??
    marker.header.frame_id = s_base_frame;
    marker.header.stamp    = ros::Time();
    marker.ns              = "my_namespace";
    marker.id              = i;
    marker.type            = visualization_msgs::Marker::CUBE;


    // Set the scale of the marker -- 0.01x0.01x0.01 here means 1cm on a side
    double d_ball_size = 0.009;

    marker.scale.x = d_ball_size;
    marker.scale.y = d_ball_size;
    marker.scale.z = d_ball_size;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration(0.1);

    // desired contacts (virtual) or real contacts
    if ( v_contact_on_link_allowed(i) || list_links[i]->contact_status == CONTACT) {
      marker.action = visualization_msgs::Marker::ADD;

      tf::poseKDLToMsg(list_links[i]->_kdl_frame, marker.pose);

      // Change color depending on status
      if (list_links[i]->contact_status == VIRTUAL_CONTACT) {
        marker.type = visualization_msgs::Marker::SPHERE;

        // Red: virtual contact
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        //////
        // ADD an arrow for contact normal
        visualization_msgs::Marker marker_arrow;

        marker_arrow.header.frame_id = s_base_frame;
        marker_arrow.header.stamp    = ros::Time();
        marker_arrow.ns              = "my_namespace";
        marker_arrow.id              = i + v_contact_on_link_allowed.rows(); // use the next integers
        marker_arrow.type            = visualization_msgs::Marker::ARROW;
        marker_arrow.lifetime = ros::Duration(0.1);


        marker_arrow.color.a = 1.0;
        marker_arrow.color.r = 0.0;
        marker_arrow.color.g = 0.0;
        marker_arrow.color.b = 1.0;

        geometry_msgs::Point point_start = marker.pose.position;
        geometry_msgs::Point point_end;

        point_end.x = list_links[i]->v_desired_cart_position(0);
        point_end.y = list_links[i]->v_desired_cart_position(1);
        point_end.z = list_links[i]->v_desired_cart_position(2);

        marker_arrow.points.push_back(point_start);
        marker_arrow.points.push_back(point_end);

        marker_arrow.scale.x = 0.002;  // shaft diameter
        marker_arrow.scale.y = 0.008; // head diameter
        marker_arrow.scale.z = 0;     // dont specify head length

        marker_array.markers.push_back(marker_arrow);

      } else if (list_links[i]->contact_status == CONTACT) {
        marker.type = visualization_msgs::Marker::SPHERE;

        // Green: contact
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.b = list_links[i]->d_pressure * (1.0/40.0);



        bool b_display_normal = true;
        if(b_display_normal){

          visualization_msgs::Marker marker_arrow_normal;

          marker_arrow_normal.header.frame_id = s_base_frame;
          marker_arrow_normal.header.stamp    = ros::Time();
          marker_arrow_normal.ns              = "my_namespace";
          marker_arrow_normal.id              = i + v_contact_on_link_allowed.rows()*2; // use the next integers
          marker_arrow_normal.type            = visualization_msgs::Marker::ARROW;
          marker_arrow_normal.lifetime = ros::Duration(0.1);


          marker_arrow_normal.color.a = 1.0;
          marker_arrow_normal.color.r = 0.0;
          marker_arrow_normal.color.g = 1.0;
          marker_arrow_normal.color.b = 0.0;

          geometry_msgs::Point point_start = marker.pose.position;
          geometry_msgs::Point point_end = point_start;

          point_end.x += list_links[i]->normalInBaseFrame()[0]*0.015;
          point_end.y += list_links[i]->normalInBaseFrame()[1]*0.015;
          point_end.z += list_links[i]->normalInBaseFrame()[2]*0.015;

          marker_arrow_normal.points.push_back(point_start);
          marker_arrow_normal.points.push_back(point_end);

          marker_arrow_normal.scale.x = 0.002;  // shaft diameter
          marker_arrow_normal.scale.y = 0.008; // head diameter
          marker_arrow_normal.scale.z = 0;     // dont specify head length

          marker_array.markers.push_back(marker_arrow_normal);



        }

        if(v_contact_on_link_allowed(i)==0){
          marker.color.r = 1.0; // Yellow contact if not allowed
        }




      } else {
        // Blue: part of allowed contacts but not activated yet
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      }
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
