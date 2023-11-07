#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
      _est(ctrlComp->estimator), _phase(ctrlComp->phase),
      _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
      _balCtrl(ctrlComp->balCtrl)
{
    _dy = ctrlComp->dy;
    _wbc = new WBC(_dy);
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;
    _tau_last.setZero();

#ifdef ROBOT_TYPE_Go1
        _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780;
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

State_Trotting::~State_Trotting()
{
    delete _gait;
}

void State_Trotting::enter()
{
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_Trotting::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else
    {
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run()
{
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcQQd();
    calcTau();
    
    if (checkStepOrNot())
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        _ctrlComp->setAllStance();
    }
    // for (int i = 0; i < 4;i++)
    // {
    //     if((*_contact)(i) == 1)
    //     {
    //         _tau.block(i * 3, 0, 3, 1) = _tau_wbc.block(i * 3, 0, 3, 1);
    //     }
    // }
    // Vec12 tau_error = _tau_wbc - _tau;
    // _tau = 0.0 * _tau + 1.0 * _tau_wbc;
    std::cout << "torque: " << _wbc->_di.block(18,0,12,1).transpose() << std::endl;
    std::cout << "tauuni: " << _tau.transpose() << std::endl
              << std::endl;
    Vec12 tau_error = _wbc->_di.block(18, 0, 12, 1) - _tau;
    std::cout << "tau_error: " << tau_error.norm() << std::endl;
    _tau = _wbc->_di.block(18, 0, 12, 1);
    // std::cout << "torque: " << _tau_wbc.transpose() << std::endl
    //           << std::endl;
    // double norm = tau_error.norm();
    // std::cout
    //     << "tua_error: " << tau_error.transpose() << std::endl
    //     << std::endl;
    // _tau = 0.9 * _tau_last + 0.1 * _tau;
    for (int i = 0; i < 12; i++)
    {
        if (_tau(i) > 33.5 || _tau(i) < -33.5)
        {
            std::cout << "OUT OF RANGE!" << std::endl;
        }
    }
    _lowCmd->setTau(_tau);
    _tau_last = _tau;

    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
        }
        else
        {
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_Trotting::checkStepOrNot()
{
    if ((fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20))
    {
        return true;
    }
    else
    { 
        return false;
        // return true; //
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz)
{
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0;
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd()
{
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Trotting::calcCmd()
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Trotting::calcTau()
{
    std::cout << std::fixed << std::setprecision(2);
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }

    /**************************************************************************************************************/
    _wbc->dynamics_consistence_task(*_contact);
    _wbc->closure_constrain_task();
    Vec2 ddr_xy;
    ddr_xy << _ddPcd(0), _ddPcd(1);
    _wbc->desired_torso_motion_task(ddr_xy);
    Vec34 swingforceFeetGlobal = _forceFeetGlobal;
    for (int i = 0; i < 4; i++)
    {
        if ((*_contact)(i) == 1)
        {
            swingforceFeetGlobal.col(i).setZero();
        }
    }
    _wbc->swing_foot_motion_task(swingforceFeetGlobal, *_contact);
    double yaw_acc = _dWbd(2);
    double height_acc = _ddPcd(2);
    _wbc->body_yaw_height_task(yaw_acc, height_acc);
    double roll_acc = _dWbd(0);
    double pitch_acc = _dWbd(1);
    _wbc->body_roll_pitch_task(roll_acc, pitch_acc);
    _wbc->torque_limit_task();
    _wbc->friction_cone_task(*_contact);
    _wbc->solve_HOproblem();
    // std::cout <<"torque: "<< _wbc->_qdd_torque.block(18,0,12,1).transpose() << std::endl;
    // Vec18 qdd;
    // Vec34 footforce_foot;
    // Vec12 torque_inv;
    // Vec3 temp1;
    // temp1 << 0, 0, -30;
    // for (int i = 0; i < 4; i++)
    // {
    //     footforce_foot.col(i) = -_wbc->_dy->_ref_R_s[i].transpose() * temp1;
    // }

    // qdd.setZero(18, 1);
    // qdd.block(0, 0, 3, 1) = Vec3(0.1, 0.1, 1.0).asDiagonal() * _dWbd;
    // qdd.block(3, 0, 3, 1) = Vec3(1.0, 1.0, 4.0).asDiagonal() * _ddPcd;
    // Vec3 q_cur, qd_cur;
    // Mat3 Kpsw, Kdsw, Kpst, Kdst;
    // Kpsw = Vec3(10, 20, 10).asDiagonal();
    // Kdsw = Vec3(1, 2, 1).asDiagonal();
    // Kpst = Vec3(1.0, 1.0, 1.0).asDiagonal();
    // Kdst = Vec3(1.0, 1.0, 1.0).asDiagonal();
    // for (int i = 0; i < 4; i++)
    // {
    //     q_cur(0) = _dy->_q[i * 3];
    //     q_cur(1) = _dy->_q[i * 3 + 1];
    //     q_cur(2) = _dy->_q[i * 3 + 2];
    //     qd_cur(0) = _dy->_dq[i * 3];
    //     qd_cur(1) = _dy->_dq[i * 3 + 1];
    //     qd_cur(2) = _dy->_dq[i * 3 + 2];
    //     if((*_contact)(i) == 1)
    //     {
    //         qdd.block(6 + 3 * i, 0, 3, 1) = Kpst * (_qGoal.col(i) - q_cur) + Kdst * (_qdGoal.col(i) - qd_cur);
    //     }
    //     else
    //     {
    //         qdd.block(6 + 3 * i, 0, 3, 1) = Kpsw * (_qGoal.col(i) - q_cur) + Kdsw * (_qdGoal.col(i) - qd_cur);
    //     }
    // }

    // std::cout << "qdd: " << qdd.transpose() << std::endl;

    /*******************************************************************************************************************/
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    // for (int i = 0; i < 4; i++)
    // {
    //     footforce_foot.col(i) = -_wbc->_dy->_ref_R_s[i].transpose() * _forceFeetBody.col(i);
    // }
    // torque_inv = _wbc->inverse_dynamics(qdd, -_forceFeetBody, *_contact);
    // std::cout << "force: " << std::endl
    //           << _forceFeetBody << std::endl;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
    // _tau_wbc = torque_inv;
    // Vec12 footuni = vec34ToVec12(-_forceFeetBody);
    // std::cout << "footuni: " << footuni.transpose() << std::endl;
    
}

void State_Trotting::calcQQd()
{
    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int i(0); i < 4; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}