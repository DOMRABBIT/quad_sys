#include "visual_rviz.h"

void rviz_link(Eigen::Matrix3d R,
               Eigen::Vector3d p,
               visualization_msgs::Marker &link,
               std::string ns,
               int id,
               std::string frame,
               ros::Publisher &marker_pub)
{
    Eigen::Quaterniond wxyz(R);
    wxyz.normalize();
    link.header.frame_id = frame;
    link.header.stamp = ros::Time::now();
    link.ns = ns;
    link.id = id;

    link.type = visualization_msgs::Marker::CYLINDER;
    link.action = visualization_msgs::Marker::ADD;

    link.pose.position.x = p(0, 0);
    link.pose.position.y = p(1, 0);
    link.pose.position.z = p(2, 0);
    link.pose.orientation.x = wxyz.x();
    link.pose.orientation.y = wxyz.y();
    link.pose.orientation.z = wxyz.z();
    link.pose.orientation.w = wxyz.w();

    link.scale.x = 0.05;
    link.scale.y = 0.05;
    link.scale.z = 0.05;

    if (id % 2 == 0)
    {
        link.color.r = 1.0f;
        link.color.g = 0.0f;
        link.color.b = 0.0f;
        link.color.a = 0.4;
    }
    else
    {
        link.color.r = 0.0f;
        link.color.g = 0.0f;
        link.color.b = 1.0f;
        link.color.a = 0.5;
    }

    link.lifetime = ros::Duration();
    marker_pub.publish(link);
}

void rviz_com(Eigen::Matrix3d R,
              Eigen::Vector3d p,
              visualization_msgs::Marker &com,
              std::string ns,
              int id,
              std::string frame,
              ros::Publisher &marker_pub)
{
    Eigen::Quaterniond wxyz(R);
    wxyz.normalize();
    com.header.frame_id = frame;
    com.header.stamp = ros::Time::now();
    com.ns = ns;
    com.id = id;

    com.type = visualization_msgs::Marker::SPHERE;
    com.action = visualization_msgs::Marker::ADD;

    com.pose.position.x = p(0, 0);
    com.pose.position.y = p(1, 0);
    com.pose.position.z = p(2, 0);
    com.pose.orientation.x = wxyz.x();
    com.pose.orientation.y = wxyz.y();
    com.pose.orientation.z = wxyz.z();
    com.pose.orientation.w = wxyz.w();

    com.scale.x = 0.02;
    com.scale.y = 0.02;
    com.scale.z = 0.02;

    if (id % 2 == 0)
    {
        com.color.r = 1.0f;
        com.color.g = 0.0f;
        com.color.b = 0.0f;
        com.color.a = 1.0;
    }
    else
    {
        com.color.r = 0.0f;
        com.color.g = 0.0f;
        com.color.b = 1.0f;
        com.color.a = 1.0;
    }

    com.lifetime = ros::Duration();
    marker_pub.publish(com);
}