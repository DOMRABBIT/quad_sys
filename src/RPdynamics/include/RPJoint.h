#ifndef RP_JOINT_H
#define RP_JOINT_H

#include "MathType.h"
#include "RPenum.h"

class Joint
{
public:
    Joint() {}
    Joint(JointType jtype, double* Init_value);
    ~Joint() {}
    void set_type_val(JointType jtype, double *Init_value);
    void set_rpy_xyz(Mat3 rpy, Vec3 xyz);

    JointType _jtype;
    MatX _S_Body;
    MatX _T_Body;
    double* _Init_value;
    double _DOF;
    Mat3 _rpyMat;
    Vec3 _xyz;

private:
};


#endif // RP_JOINT_H