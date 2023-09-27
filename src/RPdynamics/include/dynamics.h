#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "RPRobot.h"
#include "MathType.h"
#include <eigen3/Eigen/Dense>
#include "Spatial.h"

class Dynamics
{
    public:
    Dynamics(a1Robot *robot):_robot(robot)
    {
        _NB = robot->_NB;
        _NL = robot->_NL;
        _q = robot->_q;
        _dq = robot->_dq;

        _parent = _robot->_parent;

        _X_dwtree = robot->X_dwtree;
        _X_uptree = robot->X_uptree;

        _v = new Vec6[robot->_NB];
        _avp = new Vec6[robot->_NB];
        _f = new Vec6[robot->_NB];
        _a = new Vec6[robot->_NB];

        _gra.setZero();

        _base = robot->_base;
        _body = robot->_body;
        _joint = robot->_joint;
        _lpjoint = robot->_lpjoint;

        _quat_xyz = robot->_quat_xyz;
    }
    ~Dynamics(){}

    void UpdateDatafromRobot()
    {
        _robot->Update_Model();
        _X_dwtree = _robot->X_dwtree;
        _X_uptree = _robot->X_uptree;
    }
    MatX inverse_dynamic_FixedBase(double _qdd[],
                                   bool Gra_offset);

    a1Robot *_robot;
    double *_q;
    double *_dq;

    Mat6 *_X_dwtree; // body(i) coordinate respect to body(i-1) coordinate
    Mat6 *_X_uptree; // body(i-1) coordinate respect to body(i) coordinate

    Vec6 *_v;
    Vec6 *_a;
    Vec6 *_avp;
    Vec6 *_f;

    Vec6 _gra;
    int _NB, _NL;

    Base *_base;
    Body *_body;
    Joint *_joint;
    LoopJoint *_lpjoint;
    double *_quat_xyz;
    int *_parent;


private:

};

#endif