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

    JointType _jtype;
    MatX _S_Body;
    MatX _T_Body;
    double* _Init_value;
    double _DOF;
    int suc;
    int pre;

private:
};


#endif // RP_JOINT_H