#include "RPBody.h"
#include "RPJoint.h"
#include "RPRobot.h"
#include "Build_A1.h"
#include "visual_rviz.h"
#include "dynamics.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>
#include "sensor_msgs/JointState.h"
#include <iomanip>
#include <csignal>
#include <sched.h>

using namespace std;

#define FR_hip 0
#define FR_thigh 1
#define FR_calf 2
#define FL_hip 3
#define FL_thigh 4
#define FL_calf 5
#define RR_hip 6
#define RR_thigh 7
#define RR_calf 8
#define RL_hip 9
#define RL_thigh 10
#define RL_calf 11

unitree_legged_msgs::LowCmd _lowCmd;
unitree_legged_msgs::LowState _lowState;
ros::Subscriber servo_sub[12], imu_sub;
ros::Publisher servo_pub[12];
float tau_[12];
double q[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
double dq[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

bool running = true;

void show_model_in_rviz(Robot *rb, ros::Publisher &marker_pub);


void imuCallback(const sensor_msgs::Imu &msg)
{
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

void sendCmd()
{
    for (int i(0); i < 12; ++i)
    {
        _lowCmd.motorCmd[i].mode = 10;
        _lowCmd.motorCmd[i].q = 0;
        _lowCmd.motorCmd[i].dq = 0;
        _lowCmd.motorCmd[i].tau = tau_[i];
        _lowCmd.motorCmd[i].Kd = 0;
        _lowCmd.motorCmd[i].Kp = 0;
    }
    for (int m(0); m < 12; ++m)
    {
        servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

int main(int argc, char *argv[])
{
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);
    ros::init(argc, argv, "dynamic_test");
    ros::NodeHandle nm;
    ros::Rate r(500);
    ros::Publisher marker_pub = nm.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    imu_sub = nm.subscribe("/trunk_imu", 1, imuCallback);
    servo_sub[0] = nm.subscribe("/a1_gazebo/FR_hip_controller/state", 1, &FRhipCallback);
    servo_sub[1] = nm.subscribe("/a1_gazebo/FR_thigh_controller/state", 1, &FRthighCallback);
    servo_sub[2] = nm.subscribe("/a1_gazebo/FR_calf_controller/state", 1, &FRcalfCallback);
    servo_sub[3] = nm.subscribe("/a1_gazebo/FL_hip_controller/state", 1, &FLhipCallback);
    servo_sub[4] = nm.subscribe("/a1_gazebo/FL_thigh_controller/state", 1, &FLthighCallback);
    servo_sub[5] = nm.subscribe("/a1_gazebo/FL_calf_controller/state", 1, &FLcalfCallback);
    servo_sub[6] = nm.subscribe("/a1_gazebo/RR_hip_controller/state", 1, &RRhipCallback);
    servo_sub[7] = nm.subscribe("/a1_gazebo/RR_thigh_controller/state", 1, &RRthighCallback);
    servo_sub[8] = nm.subscribe("/a1_gazebo/RR_calf_controller/state", 1, &RRcalfCallback);
    servo_sub[9] = nm.subscribe("/a1_gazebo/RL_hip_controller/state", 1, &RLhipCallback);
    servo_sub[10] = nm.subscribe("/a1_gazebo/RL_thigh_controller/state", 1, &RLthighCallback);
    servo_sub[11] = nm.subscribe("/a1_gazebo/RL_calf_controller/state", 1, &RLcalfCallback);
    // ros::Subscriber subscriber = nm.subscribe("/a1_gazebo/joint_states", 12, &subscriberCallback);

    servo_pub[0] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nm.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    a1Robot *a1 = new a1Robot();
    Dynamics *dy = new Dynamics(a1);

    // a1->Update_Model();
    double qdd[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    double q_desire[12] = {0.2f, 0.5f, -1.0f, -0.2f, 0.5f, -1.0f, 0.2f, 0.5f, -1.0f, -0.2f, 0.5f, -1.0f};
    double quaxyz[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    double qdd18[18] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    double v_base[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    MatX qdd18v = Eigen::Map<MatX>(qdd18, 18, 1);
    MatX tau;
    tau.setZero(12, 1);
    // cout << a1->_q[0] << endl;
    signal(SIGINT, ShutDown);
    while (ros::ok() && running)
    {
        for (int i = 0; i < 12;i++)
        {
            q[i] = _lowState.motorState[i].q;
            dq[i] = _lowState.motorState[i].dq;
        }
        a1->set_q(q);
        a1->set_dq(dq);
        a1->set_quaxyz(quaxyz);
        a1->set_vbase(v_base);
        a1->Update_Model();
        tau = dy->inverse_dynamic_FixedBase(qdd, true);
        Eigen::MatrixXd K, k, C, H_RNEA, H_CRBA, H_FLT, H_fl,C_FLT;
        K = dy->Cal_K_Flt(k);
        Eigen::FullPivLU<MatX> lu(K);
        MatX G = lu.kernel();
        Eigen::Matrix<double, 12, 1> lenda;
        lenda << 0, 0, 33.5, 0, 0, 33.5, 0, 0, 33.5, 0, 0, 33.5;
        Eigen::Matrix<double, 3, 4> Ffoot;
        for (int i = 0; i < 4; i++)
        {
            Ffoot.block(0, i, 3, 1) = dy->_ref_R_s[i] * lenda.segment(i * (3), 3);
        }
        a1->Update_Model();
        C = dy->Cal_Generalize_Bias_force(true);
        H_RNEA = dy->Cal_Generalize_Inertial_Matrix_RNEA(C);
        H_CRBA = dy->Cal_Generalize_Inertial_Matrix_CRBA();
        H_FLT = dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(H_fl);
        C_FLT = dy->Cal_Generalize_Bias_force_Flt(true);
        MatX tau_ext;
        tau_ext.setZero(18, 1);
        MatX jaco;
        jaco = dy->Cal_Geometric_Jacobain(2,Coordiante::INERTIAL);
        
        // cout
        //     << "Ffoot in world coordinate: " << endl;
        // cout <<  Ffoot << endl;
        // cout << "K: " << endl
        //      << lenda.transpose() * K << endl;
        // cout
        //     << H_FLT * qdd18v + C_FLT << endl
        //     << endl;
        // for (int i = 0; i < a1->_NB; i++)
        // {
        //     std::cout << a1->X_dwtree[i] << endl;
        // }
        // cout << k.rows() << endl
        //      << endl;
        // show_model_in_rviz(a1, marker_pub);
        // cout << tau.transpose() << endl;
        for (int i(0); i < 12; ++i)
        {
            _lowCmd.motorCmd[i].mode = 10;
            _lowCmd.motorCmd[i].q = q_desire[i];
            _lowCmd.motorCmd[i].dq = 0;
            _lowCmd.motorCmd[i].tau = tau(i);
            _lowCmd.motorCmd[i].Kd = 2;
            _lowCmd.motorCmd[i].Kp = 20;
        }
        for (int m(0); m < 12; ++m)
        {
            servo_pub[m].publish(_lowCmd.motorCmd[m]);
        }

        a1->_isUpdated = false;
        ros::spinOnce();
        r.sleep();
        
    }

    delete dy;
    delete a1;
    

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