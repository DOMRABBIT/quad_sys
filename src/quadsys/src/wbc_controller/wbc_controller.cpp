#include "wbc_controller/wbc_controller.h"

void WBC::dynamics_consistence_task(VecInt4 contact)
{
    int contact_num = 0;
    int row_index = 0;
    MatX K_temp, k_temp;
    // calculate general inertial matrix
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    // calculate general bias force
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    // calculate constrain matrix K and k
    K_temp = _dy->Cal_K_Flt(k_temp);
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    _K.setZero(contact_num * 3, 18);
    _k.setZero(contact_num * 3, 1);
    for (int i = 0; i < 4;i++)
    {
        if(contact(i) == 1)
        {
            _K.block(3 * row_index, 0, 3, 18) = K_temp.block(3 * i, 0, 3, 18);
            _k.block(3 * row_index, 0, 3, 1) = k_temp.block(3 * i, 0, 3, 1);
            row_index++;
        }
    }
    // calculate nullspace matrix of K
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
void WBC::swing_foot_motion_task(Vec34 swing_acc,VecInt4 contact)
{
    MatX A, b;
    Vec3 swing_acc_in_suc;
    int swing_num = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            swing_num++;
    }
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
            avp = _dy->_ref_X_s[i] * _dy->_avp[2 + 3 * i];
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

// contact is the contact situation of four foot 1: contact 0: swing
void WBC::friction_cone_task(VecInt4 contact)
{
    MatX D, f;
    int row_index = 0;
    MatX B;
    MatX ref_R_s_ext;
    int contact_num = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }

    D.setZero(5 * contact_num, 30);
    f.setZero(5 * contact_num, 1);
    B.setZero(5 * contact_num, 3 * contact_num);
    ref_R_s_ext.setIdentity(contact_num * 3, contact_num * 3);
    for (int i = 0; i < contact_num; i++)
    {
        B.block(5 * i, 3 * i, 5, 3) = _Ffri;
    }
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            ref_R_s_ext.block(row_index * 3, row_index * 3, 3, 3) = _dy->_ref_R_s[i];
            row_index++;
        }
    }
    MatX KK_T = _K * _K.transpose();
    MatX K_leftinv = KK_T.inverse() * _K;
    MatX D_temp;
    D_temp.setZero(18, 30);
    D_temp.block(0, 0, 18, 18) = _H;
    D_temp.block(0, 18, 18, 12) = -_S.transpose();
    MatX BRK_inv;
    BRK_inv.setZero(5 * contact_num, 18);
    BRK_inv = B * ref_R_s_ext * K_leftinv;
    D = BRK_inv * D_temp;
    f = BRK_inv * _C;
    _D = D;
    _f = f;

    _ineq_task = new ineq_Task(D, f);
}

Vec12 WBC::inverse_dynamics(Vec18 qdd, Vec34 footforce, VecInt4 contact)
{
    /****************************************************************/
    contact << 1, 1, 1, 1;
    MatX S_T;
    S_T.setZero(18, 12);
    S_T.block(6, 0, 12, 12).setIdentity(12, 12);
    _S.setZero(12, 18);
    _S.block(0, 6, 12, 12).setIdentity(12, 12);
    /****************************************************************/
    Vec12 torque;
    _H = _dy->Cal_Generalize_Inertial_Matrix_CRBA_Flt(_H_fl);
    _C = _dy->Cal_Generalize_Bias_force_Flt(true);
    MatX smallC = _dy->Cal_Generalize_Bias_force(true);
    Vec12 errorC = smallC - _C.block(6, 0, 12, 1);
    // double norm_C = errorC.norm();
    // if (norm_C>0.01)
    // std::cout << "C_error: " << errorC.transpose() << std::endl;
    MatX K_temp, k_temp, lenda;
    K_temp = _dy->Cal_K_Flt(k_temp);
    int contact_num = 0;
    int row_index = 0;
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
            contact_num++;
    }
    _K.setZero(contact_num * 3, 18);
    _k.setZero(contact_num * 3, 1);
    lenda.setZero(contact_num * 3, 1);
    for (int i = 0; i < 4; i++)
    {
        if (contact(i) == 1)
        {
            _K.block(3 * row_index, 0, 3, 18) = K_temp.block(3 * i, 0, 3, 18);
            _k.block(3 * row_index, 0, 3, 1) = k_temp.block(3 * i, 0, 3, 1);
            lenda.block(3 * row_index, 0, 3, 1) = footforce.col(i);
            row_index++;
        }
    }
    // friction_cone_task(contact);
    MatX A, b;
    A.setZero(18, 24);
    A.block(0, 0, 18, 12) = _S.transpose(); //_S.transpose()
    A.block(0, 12, 18, 12) = _K.transpose();
    b = _H * qdd + _C;

    // std::cout << "_C " << std::endl
    //           << _C.transpose() << std::endl;
    MatX D, f;
    Eigen::Matrix<double, 12, 12> bigR;
    Eigen::Matrix<double, 20, 12> bigF_fri;
    for (int i = 0; i < 4; i++)
    {
        bigR.block(i * 3, i * 3, 3, 3) = _dy->_ref_R_s[i];
        bigF_fri.block(5 * i, 3 * i, 5, 3) = _Ffri;
    }
    D.setZero(20, 24);
    D.block(0, 12, 20, 12) = bigF_fri * bigR;
    f.setZero(20, 1);
    solve_QProblem(A, b, D, f);
    Eigen::FullPivLU<MatX> lu(A);
    _di = lu.solve(b);
    MatX null_A = lu.kernel();
    // std::cout << "A_size: " << null_A.rows() << " " << null_A.cols() << std::endl;
    // std::cout << "size_di: " << _di.size() << std::endl;
    VecX w_ = solve_QProblem_Ab(null_A, -_di);

    VecX resultX = null_A * w_ + _di;
    Vec12 footf;
    // for (int i = 0; i < 12; i++)
    // {
    //     torque[i] = _di[i];
    // }
    // for (int i = 0; i < 12; i++)
    // {
    //     footf[i] = _di[i + 12];
    // }
    for (int i = 0; i < 12; i++)
    {
        torque[i] = resultX[i];
    }
    for (int i = 0; i < 12; i++)
    {
        footf[i] = resultX[i + 12];
    }
    MatX error = A * _di - b;
    Vec12 footuni = vec34ToVec12(footforce);
    MatX force_temp = _K.transpose() * footf;
    std::cout << "error: " << error.transpose() << std::endl;
    // std::cout << "KTlda: " << force_temp.transpose() << std::endl;
    std::cout << "footfmy: " << footf.transpose() << std::endl;
    return torque;
}

void WBC::solve_HOproblem()
{
    MatX C_bar, d_bar, A_bar;
    MatX b_bar;
    C_bar.setIdentity(30, 30);
    d_bar.setZero(30, 1);
    b_bar.setZero(30, 1);
    int maxtask = -1;
    for (int i = 0; i < 7; i++)
    {
        A_bar = _eq_task[i]->_A * C_bar;
        b_bar = _eq_task[i]->_b - _eq_task[i]->_A * d_bar;
        solve_QProblem(A_bar, b_bar, _ineq_task->_D, _ineq_task->_f);
        d_bar = d_bar + C_bar * _di;
        Eigen::FullPivLU<MatX> lu(A_bar);
        if (lu.rank() >= A_bar.cols())// if A_bar full rank
        {
            _qdd_torque = d_bar;
            maxtask = i;
            break;
        }
        MatX null_A = lu.kernel();
        C_bar = C_bar * null_A;
    }
    // std::cout << "max_task: " << maxtask << std::endl;

    _qdd_torque = d_bar;
}

void WBC::solve_QProblem(MatX A, MatX b, MatX D, MatX f)
{
    int n = A.cols();
    int m = 0;
    int p = f.size(); // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    _di.setZero(n, 1);
    _min_ident.setIdentity(n, n);
    _min_ident = _min_ident * 0.00001f;
    _G0 = A.transpose() * A + _min_ident;
    _g0 = A.transpose() * (-b);
    _CI = D;
    _ci0 = f;
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }


    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < p; ++j)
        {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }

    for (int i = 0; i < p; ++i)
    {
        ci0[i] = _ci0(i);
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // std::cout << "min value: " << value << std::endl;

    for (int i = 0; i < n; ++i)
    {
        _di[i] = x[i];
    }
}

VecX WBC::solve_QProblem_Ab(MatX A, MatX b)
{
    VecX result;
    int n = A.cols();
    int m = 0;
    int p = 0; // f.size()
    _G0.setZero(n, n);
    _g0.setZero(n, 1);
    // _min_ident.setIdentity(n, n);
    // _min_ident = _min_ident * 0.0001f;
    _G0 = A.transpose() * A;
    _g0 = A.transpose() * (-b);
    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            G[i][j] = _G0(i, j);
        }
    }

    for (int i = 0; i < n; ++i)
    {
        g0[i] = _g0(i);
    }


    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    // // std::cout << "min value: " << value << std::endl;
    result.setZero(n, 1);
    for (int i = 0; i < n; ++i)
    {
        result(i) = x[i];
    }
    return result;
}
