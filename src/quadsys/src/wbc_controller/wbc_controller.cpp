#include "wbc_controller/wbc_controller.h"

void WBC::dynamics_consistence_task()
{
    // calculate general inertial matrix
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // calculate general bias force
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // calculate constrain matrix K and k
    _K = _dy->Cal_K_Flt(_k);
    //calculate nullspace matrix of K
    Eigen::FullPivLU<MatX> lu(_K);
    _G = lu.kernel();

    // dynamics constrain A and b
    MatX A, b;
    int cols, rows;
    rows = _G.rows(); // G : rows x cols   G^T : cols x rows
    cols = _G.cols();
    A.setZero(cols, 30);
    b.setZero(cols, 1);
    _S.setZero(12, 18);
    _S.block(0, 6, 12, 12).setIdentity(12, 12);
    A.block(0, 0, cols, 18) = _G.transpose() * _H;
    A.block(0, 18, cols, 12) = -_G.transpose() * _S.transpose();

    b = -_G.transpose() * _C;

    _eq_task[0] = new eq_Task(A, b);
}

void WBC::closure_constrain_task()
{
    MatX A, b;
    int cols, rows;
    rows = _K.rows(); // K : rows x cols 
    cols = _K.cols();
    A.setZero(rows, 30);
    A.block(0, 0, rows, 18) = _K;
    b = _k;

    _eq_task[1] = new eq_Task(A, b);
}

void WBC::desired_torso_motion_task(Vec2 ddr_xy)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    A.block(0, 0, 2, 18) = _I_xy;
    b = ddr_xy;

    _eq_task[2] = new eq_Task(A, b);
}

// swing_acc only contain swing foot acceleration, which means 
// the swing cols of swing_acc must set zero when using this function
void WBC::swing_foot_motion_task(Vec34 swing_acc,int swing_num)
{
    MatX A, b;
    Vec3 swing_acc_in_suc;
    A.setZero(swing_num * 3, 30);
    b.setZero(swing_num * 3, 1);
    int row_index = 0;
    _J_swingfoot.setZero(swing_num * 3, 18);
    _J_swingfoot.block(0, 0, 3, 6) = _dy->_robot->_base->_fltjoint->_X_Base2Wrd.block(3, 0, 3, 6);
    _J_swingfoot.block(3, 0, 3, 6) = _J_swingfoot.block(0, 0, 3, 6);
    _dy->_robot->update_footEnd();
    
    Mat3 px[4];
    for (int i = 0; i < 4;i++)
    {
        Vec3 footend = _dy->_robot->_endPfoot.block(0,i,3,1);
        px[i] << 0,         -footend(2), footend(1),
                 footend(2), 0,         -footend(0),
                -footend(1), footend(0), 0;
    }

    Eigen::Matrix<double, 3, 6> PIMat;
    Vec6 avp;
    PIMat.block(0, 3, 3, 3).setIdentity(3, 3);
    for (int i = 0; i < 4; i++)
    {
        if (swing_acc.block(0, i, 3, 1).norm() != 0)
        {
            PIMat.block(0, 0, 3, 3) = -px[i];
            _J_swingfoot.block(row_index * 3, 6 + 3 * i, 3, 3) = PIMat * _dy->Cal_Geometric_Jacobain(2+3*i, Coordiante::INERTIAL).block(0, 3 * i, 6, 3);
            avp = _dy->_ref_X_s * _dy->_avp[2 + 3 * i];
            b.block(row_index * 3, 0, 3, 1) = swing_acc.block(0, i, 3, 1) - avp.block(3,0,3,1);
            row_index++;
        }
    }

    A.block(0, 0, swing_num * 3, 18) = _J_swingfoot;

    _eq_task[3] = new eq_Task(A, b);
}

void WBC::body_yaw_height_task(double yaw_acc, double height_acc)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    Vec2 yaw_height;
    yaw_height << yaw_acc, height_acc;
    A.block(0, 0, 2, 18) = _I_yaw_height;
    b = yaw_height;

    _eq_task[4] = new eq_Task(A, b);
}

void WBC::body_roll_pitch_task(double roll_acc, double pitch_acc)
{
    MatX A, b;
    A.setZero(2, 30);
    b.setZero(2, 1);
    Vec2 roll_pitch;
    roll_pitch << roll_acc, pitch_acc;
    A.block(0,0,2,18) = _I_roll_pitch;
    b = roll_pitch;

    _eq_task[5] = new eq_Task(A, b);
}

void WBC::torque_limit_task()
{
    MatX A, b;
    A.setZero(12, 30);
    b.setZero(12, 1);
    A.block(0, 18, 12, 12) = _I_torque;

    _eq_task[6] = new eq_Task(A, b);
}

void WBC::friction_cone_task()
{
    
}
