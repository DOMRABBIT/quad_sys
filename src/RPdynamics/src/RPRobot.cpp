#include "RPRobot.h"
#include "Spatial.h"

Robot::Robot(int NB, int NL)
:_NB(NB), _NL(NL)
{
    _body = (Body *)malloc(sizeof(Body) * _NB);
    _joint = (Joint *)malloc(sizeof(Joint) * _NB);
    _parent = (int *)malloc(sizeof(int) * _NB);
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
        _parent[i] = i;
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
        _lpjoint[i]._suc = -1;
        _lpjoint[i]._pre = -1;
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
    _parent = (int *)malloc(sizeof(int) * _NB);
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
        _parent[i] = i;
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
    if(_base->get_BaseType() == BaseType::Floating)
    {
        Mat4 T = Flt_Transform();
        _base->_fltjoint->_T_Base2Wrd = T;
        AdjointT(T, _base->_fltjoint->_X_Base2Wrd);
        T.transposeInPlace();
        _base->_fltjoint->_T_Wrd2Base = T;
        AdjointT(T, _base->_fltjoint->_X_Wrd2Base);
    }
    else
    {
        _base->_fltjoint->_T_Base2Wrd.setIdentity();
        _base->_fltjoint->_T_Wrd2Base.setIdentity();
        _base->_fltjoint->_X_Base2Wrd.setIdentity();
        _base->_fltjoint->_X_Wrd2Base.setIdentity();
    }

    for (int i = 0; i < _NB; i++)
    {
        if (_joint[i]._jtype == JointType::RZ)
        {
            Xq[i] = rotz(_q[i]); // X_down
            Tq[i] = roz(_q[i]);  // T_down
        }
        else if (_joint[i]._jtype == JointType::RX)
        {
            Xq[i] = rotx(_q[i]); // X_down
            Tq[i] = rox(_q[i]);  // T_down
        }
        else if (_joint[i]._jtype == JointType::RY)
        {
            Xq[i] = roty(_q[i]); // X_down
            Tq[i] = roy(_q[i]);  // T_down
        }

        X_dwtree[i] = Xj[i] * Xq[i];
        T_dwtree[i] = Tj[i] * Tq[i];

        Mat3 R;
        Vec3 xyz;
        R = T_dwtree[i].block(0, 0, 3, 3).transpose();
        xyz = (-R) * T_dwtree[i].block(0, 3, 3, 1);
        Rp2T(R,xyz,T_uptree[i]);
        AdjointT(T_uptree[i], X_uptree[i]);

        // std::cout << i << ": " << endl
        //           << model.Ttree[i] << endl;
    }
}

MatX Robot::Cal_Jacobian(int ib, Coordiante frame)
{
    MatX J;
    J.setZero(6, _NB); // initialize Jacobian matrix

    // initial k set
    int *k, *k_temp;
    k_temp = (int *)malloc(sizeof(int) * _NB);
    k = (int *)malloc(sizeof(int) * _NB);
    int j = ib;
    int num = 0;
    while (j >= 0)
    {
        k_temp[num] = j;
        j = _parent[j];
        num++;
    }
    for (int i = num - 1; i >= 0; --i)
    {
        k[num - 1 - i] = k_temp[i];
    }
    // Jacobian of body ib repesent at base frame
    if (frame == Coordiante::BASE) // base numbered -1
    {
        Mat6 X_down;
        X_down.setIdentity(6, 6);
        for (int i = 0; i < num; i++)
        {
            X_down = X_down * X_dwtree[k[i]];
            J.block(0, k[i], 6, 1) = X_down * _joint[k[i]]._S_Body;
        }
    }
    // Jacobian of body ib repesent at body ib coordinate
    else if (frame == Coordiante::BODY)
    {
        Mat6 X_up;
        X_up.setIdentity(6, 6);
        J.block(0, k[num - 1], 6, 1) = _joint[k[num - 1]]._S_Body;
        for (int i = num - 1; i > 0; --i)
        {
            X_up = X_up * X_uptree[i];
            J.block(0, k[i - 1], 6, 1) = X_up * _joint[k[i - 1]]._S_Body;
        }
    }
    return J;
}

// q(0:3):quaternion
// q(4:6):xyz
Mat4 Robot::Flt_Transform()
{
    Mat4 T;
    Mat3 R;
    Vec3 xyz;
    xyz << _quat_xyz[4], _quat_xyz[5], _quat_xyz[6];

    Eigen::Quaterniond qua(_quat_xyz[0],
                           _quat_xyz[1], 
                           _quat_xyz[2], 
                           _quat_xyz[3]); // w x y z
    qua.normalize();
    R = qua.matrix();
    T.setIdentity(4, 4);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = xyz;
    return T;
}