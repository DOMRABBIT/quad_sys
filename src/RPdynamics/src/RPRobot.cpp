#include "RPRobot.h"

Robot::Robot(int NB, int NL)
:_NB(NB), _NL(NL)
{
    _body = (Body *)malloc(sizeof(Body) * _NB);
    _joint = (Joint *)malloc(sizeof(Joint) * _NB);
    parent = (int *)malloc(sizeof(int) * _NB);
    X_dwtree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    X_uptree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xj = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xq = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    T_dwtree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    T_uptree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tj = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tq = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    _q = (double *)malloc(sizeof(double) * _NB);
    _dq = (double *)malloc(sizeof(double) * _NB);

    _lpjoint = (LoopJoint *)malloc(sizeof(LoopJoint) * _NL);

    for (int i = 0; i < _NB; i++)
    {
        parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
    }

    for (int i = 0; i < NL; i++)
    {
        _lpjoint[i].suc = -1;
        _lpjoint[i].pre = -1;
        _lpjoint[i].Xp.setIdentity(6, 6);
        _lpjoint[i].Xs.setIdentity(6, 6);
    }
    _isUpdated = false;
    _systick = getSystemTime();
}

Robot::Robot(int NB)
    : _NB(NB)
{
    _NL = 0;
    _body = (Body *)malloc(sizeof(Body) * _NB);
    _joint = (Joint *)malloc(sizeof(Joint) * _NB);
    parent = (int *)malloc(sizeof(int) * _NB);
    X_dwtree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    X_uptree = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xj = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    Xq = (Mat6 *)malloc(sizeof(Mat6) * _NB);
    T_dwtree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    T_uptree = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tj = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    Tq = (Mat4 *)malloc(sizeof(Mat4) * _NB);
    _q = (double *)malloc(sizeof(double) * _NB);
    _dq = (double *)malloc(sizeof(double) * _NB);

    for (int i = 0; i < _NB; i++)
    {
        parent[i] = i;
        X_dwtree[i] = Mat6::Identity(6, 6);
        X_uptree[i] = Mat6::Identity(6, 6);
        Xj[i] = Mat6::Identity(6, 6);
        Xq[i] = Mat6::Identity(6, 6);
        T_dwtree[i] = Mat4::Identity(4, 4);
        T_uptree[i] = Mat4::Identity(4, 4);
        Tj[i] = Mat4::Identity(4, 4);
        Tq[i] = Mat4::Identity(4, 4);
    }
    _isUpdated = false;
    _systick = getSystemTime();
}

void Robot::Forward_Kinematic()
{
    if(_base.get_BaseType() == BaseType::Floating)
    {
        
    }

}

MatX Robot::Cal_Jacobian(int num, Coordiante frame)
{

}

// q(0:3):quaternion
// q(4:6):xyz
Mat4 Robot::Flt_Transform(double q[])
{
    Mat4 T;
    Mat3 R;
    Vec3 xyz;
    xyz << q[4], q[5], q[6];

    Eigen::Quaterniond qua(q[0], q[1], q[2], q[3]); // w x y z
    qua.normalize();
    R = qua.matrix();
    T.setIdentity(4, 4);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = xyz;
    return T;
}