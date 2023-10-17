#ifndef _WBC_CONTROLLER_H_
#define _WBC_CONTROLLER_H_

#include "dynamics.h"
#include "MathType.h"

class eq_Task
{
    public:
        eq_Task(MatX A, VecX b)
        {
            _A = A;
            _b = b;
        }
        ~eq_Task(){}
        MatX _A;
        VecX _b;

    private:
};

class ineq_Task
{
    public:
        ineq_Task(MatX D, VecX f)
        {
            _D = D;
            _f = f;
        }
        ~ineq_Task() {}
        MatX _D;
        VecX _f;

    private:
};

class WBC
{
    WBC(Dynamics *dy) : _dy(dy)
    {
        _I_xy.setZero(2, 18);
        _I_xy << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _I_yaw_height.setZero(2, 18);
        _I_yaw_height << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        _I_roll_pitch.setZero(2, 18);
        _I_roll_pitch << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        _I_torque.setIdentity(12, 12);
    }
    ~WBC() {}

public:
    Dynamics *_dy;
    eq_Task *_eq_task[7];
    ineq_Task *_ineq_task;
    MatX _H, _H_fl, _C, _K, _k, _S, _G, _I_xy, _J_swingfoot, _I_yaw_height, _I_roll_pitch, _I_torque;

    void dynamics_consistence_task();
    void closure_constrain_task();
    void desired_torso_motion_task(Vec2 ddr_xy);
    void swing_foot_motion_task(Vec34 swing_acc, int swing_num);
    void body_yaw_height_task(double yaw_acc,double height_acc);
    void body_roll_pitch_task(double roll_acc, double pitch_acc);
    void torque_limit_task();

    void friction_cone_task();

private:
};

#endif //_WBC_CONTROLLER_H_