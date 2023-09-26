#ifndef _VISUAL_RVIZ_H_
#define _VISUAL_RVIZ_H_

#include "ros/ros.h"
#include "MathType.h"
#include <visualization_msgs/Marker.h>

void rviz_link(Mat3 R,
               Vec3 p,
               visualization_msgs::Marker &link,
               std::string ns,
               int id,
               std::string frame,
               ros::Publisher &marker_pub);

void rviz_com(Mat3 R,
              Vec3 p,
              visualization_msgs::Marker &link,
              std::string ns,
              int id,
              std::string frame,
              ros::Publisher &marker_pub);

#endif