#include "RPBody.h"
#include "RPJoint.h"

int main()
{
    double m = 1.0;
    Vec3 com;
    com << 1, 1, 1;
    Mat3 Ic;
    Ic.setIdentity();

    Body body(m, com, Ic);
    // Vec6 value;
    // value.setZero();
    // Joint<Vec6> Joint(JointType::RX, value);

    return 0;
}