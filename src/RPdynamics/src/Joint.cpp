#include "RPJoint.h"

Joint::Joint(JointType jtype, double* Init_value)
{
    set_type_val(jtype, Init_value);
}

void Joint::set_type_val(JointType jtype, double *Init_value)
{
    _jtype = jtype;
    _Init_value = Init_value;
    switch (jtype)
    {
    case JointType::RX:
        _S_Body.setZero(6, 1);
        _S_Body
            << 1, 0, 0, 0, 0, 0;
        _T_Body.setZero(6, 5);
        _T_Body
            << 0, 0, 0, 0, 0,
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
        _DOF = 1;
        break;
    case JointType::RY:
        _S_Body.setZero(6, 1);
        _S_Body
            << 0,
            1, 0, 0, 0, 0;
        _T_Body.setZero(6, 5);
        _T_Body
            << 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
        _DOF = 1;
        break;
    case JointType::RZ:
        _S_Body.setZero(6, 1);
        _S_Body
            << 0, 0, 1, 0, 0, 0;
        _T_Body.setZero(6, 5);
        _T_Body
            << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
        _DOF = 1;
        break;
    case JointType::PX:
        _S_Body.setZero(6, 1);
        _S_Body
            << 0,  0, 0, 1, 0, 0;
        _T_Body.setZero(6, 5);
        _T_Body
            << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
        _DOF = 1;
        break;
    case JointType::PY:
        _S_Body.setZero(6, 1);
        _S_Body
            << 0, 0, 0, 0, 1, 0;
        _T_Body.setZero(6, 5);
        _T_Body
            << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 1;
        _DOF = 1;
        break;
    case JointType::PZ:
        _S_Body.setZero(6, 1);
        _S_Body
            << 0, 0, 0, 0, 0, 1;
        _T_Body.setZero(6, 5);
        _T_Body
            << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1,
            0, 0, 0, 0, 0;
        _DOF = 1;
        break;
    case JointType::UXY:
        _S_Body.setZero(6, 2);
        _S_Body
            << 1, 0,
            0, 1,
            0, 0,
            0, 0,
            0, 0,
            0, 0;
        _T_Body.setZero(6, 4);
        _T_Body
            << 0,  0, 0, 0,
            0, 0, 0, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        _DOF = 2;
        break;
    case JointType::UXZ:
        _S_Body.setZero(6, 2);
        _S_Body
            << 1,  0,
            0, 0,
            0, 1,
            0, 0,
            0, 0,
            0, 0;
        _T_Body.setZero(6, 4);
        _T_Body
            << 0,  0, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        _DOF = 2;
        break;
    case JointType::UYZ:
        _S_Body.setZero(6, 2);
        _S_Body
            << 0,  0,
            1, 0,
            0, 1,
            0, 0,
            0, 0,
            0, 0;
        _T_Body.setZero(6, 4);
        _T_Body
            << 1, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        _DOF = 2;
        break;
    case JointType::SPHERE:
        _S_Body.setZero(6, 3);
        _S_Body
            << 1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;
        _T_Body.setZero(6, 3);
        _T_Body
            << 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        _DOF = 3;
    case JointType::FLOATING:
        _S_Body.setIdentity(6, 6);
        _T_Body.setZero(6, 4);
        _DOF = 6;
    default:
        _S_Body
            << 0, 0, 1, 0, 0, 0;
        break;
    }
    for (int i = 0; i < _DOF; i++)
    {
        if (Init_value != nullptr)
            _Init_value[i] = Init_value[i];
    }
}

void Joint::set_rpy_xyz(Mat3 rpy, Vec3 xyz)
{
    _rpyMat = rpy;
    _xyz = xyz;
}