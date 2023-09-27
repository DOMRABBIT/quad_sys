#include "dynamics.h"

MatX Dynamics::inverse_dynamic_FixedBase(double _ddq[],
                               bool Gra_offset)
{
    MatX dq = Eigen::Map<MatX>(_dq, _NB, 1);
    MatX ddq = Eigen::Map<MatX>(_ddq, _NB, 1);
    MatX torque = Eigen::Map<MatX>(_q, _NB, 1); // torque is the same size of vector(_q)
    Eigen::Matrix<double, 6, 1> vJ;            // joint velocity, satisfying vJ = S(j)*qd and v(i) = v(i-1) + vJ
    torque.setZero();                                                   
    // Update Robot state, X
    _robot->Update_Model();

    if (Gra_offset)
        _gra << 0, 0, 0, 0, 0, -9.81;
    else
        _gra << 0, 0, 0, 0, 0, 0;

    // forward pass, velocity and acceleration propagation
    for (int i = 0; i < _NB; i++)
    {
        vJ = _joint[i]._S_Body * dq(i);
        if (_parent[i] == -1)
        {
            _v[i] = vJ;
            _a[i] = _X_uptree[i] * (-_gra) + _joint[i]._S_Body * ddq(i);
        }
        else
        {
            _v[i] = _X_uptree[i] * _v[_parent[i]] + vJ;
            _a[i] = _X_uptree[i] * _a[_parent[i]] + _joint[i]._S_Body * ddq(i) + crm(_v[i]) * vJ;
            _avp[i] = crm(_v[i]) * vJ;
        }
        _f[i] = _body[i]._rbi * _a[i] + crf(_v[i]) * _body[i]._rbi * _v[i];
    }

    for (int i = _NB - 1; i >= 0; --i)
    {
        torque(i) = _joint[i]._S_Body.transpose() * _f[i];

        if (_parent[i] != -1)
        {
            _f[_parent[i]] = _f[_parent[i]] + _X_uptree[i].transpose() * _f[i];
        }
    }
    return torque;
}

