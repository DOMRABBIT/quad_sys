#include "RPBody.h"
#include "RPJoint.h"
#include "RPRobot.h"
#include "Build_A1.h"
#include "visual_rviz.h"
#include "dynamics.h"

using namespace std;

void show_model_in_rviz(Robot *rb, ros::Publisher &marker_pub);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    a1Robot *a1 = new a1Robot();
    Dynamics *dy = new Dynamics(a1);
    // cout << a1->_q[0] << endl;
    // cout << a1->_q[0] << endl;
    // cout << dy->_q[0] << endl;


    double q[12] = {0.5f, 0.2f, 0.0f, 0.1f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    a1->set_q(q);

    // cout << a1->_q[0] << endl;

    while (ros::ok())
    {
        a1->Update_Model();
        show_model_in_rviz(a1, marker_pub);

        
        a1->_isUpdated = false;
        r.sleep();
    }

    return 0;


}

void show_model_in_rviz(Robot *rb, ros::Publisher &marker_pub)
{
    ros::Rate r1(1000);

    string frame = "base";
    Mat3 R;
    Vec3 p;
    Mat4 T_up;
    R.setIdentity(3, 3);
    p.setZero();

    // show base
    r1.sleep();
    visualization_msgs::Marker link;
    rviz_link(R, p, link, "base", 0, frame, marker_pub);

    // show link
    for (int i = 0; i < rb->_NB; i++)
    {
        r1.sleep();
        T_up.setIdentity(4, 4);
        int j = i;
        while (j > -1)
        {
            T_up = rb->T_dwtree[j] * T_up;
            j = rb->_parent[j];
        }
        R = T_up.block(0, 0, 3, 3);
        p = T_up.block(0, 3, 3, 1);
        rviz_link(R, p, link, "base", i + 1, frame, marker_pub);
    }

    T_up.setIdentity(4, 4);
    visualization_msgs::Marker com;
    // show center of mass
    for (int i = 0; i < rb->_NB; i++)
    {
        r1.sleep();
        T_up.setIdentity(4, 4);
        int j = i;
        while (j > -1)
        {
            T_up = rb->T_dwtree[j] * T_up;
            j = rb->_parent[j];
        }
        R = T_up.block(0, 0, 3, 3);
        p = T_up.block(0, 3, 3, 1);
        p += R * rb->_body->_com;
        rviz_com(R, p, com, "base", i + rb->_NB + 1, frame, marker_pub);
    }
}