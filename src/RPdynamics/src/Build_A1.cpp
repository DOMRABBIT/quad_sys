// #include "Build_A1.h"
// #include "Spatial.h"

// void build_a1(a1Robot *a1)
// {
//     int parentset[12] = {-1, 0, 1, -1, 3, 4, -1, 6, 7, -1, 9, 10};
//     // number the parent set
//     for (int i = 0; i < a1->_NB; i++)
//     {
//         a1->_parent[i] = parentset[i];
//     }

//     //Floating base settings
//     a1->_base->set_BaseType(BaseType::Floating);
//     a1->_base->set_mcI(a1->_urdf->_urbase);
//     a1->_base->_fltjoint = new FltJoint(a1->_urdf->_urfltjoint);
//     a1->_base->_fltjoint->_T_Base2Wrd.setIdentity();
//     a1->_base->_fltjoint->_T_Wrd2Base.setIdentity();
//     a1->_base->_fltjoint->_X_Base2Wrd.setIdentity();
//     a1->_base->_fltjoint->_X_Wrd2Base.setIdentity();
//     a1->_quat_xyz[0] = 1.0f;
//     a1->_quat_xyz[1] = 0.0f;
//     a1->_quat_xyz[2] = 0.0f;
//     a1->_quat_xyz[3] = 0.0f;
//     a1->_quat_xyz[4] = 0.0f;
//     a1->_quat_xyz[5] = 0.0f;
//     a1->_quat_xyz[6] = 0.0f;

//     // build bodys and joints
//     double initVal[6] = {0, 0, 0, 0, 0, 0};
//     /**-----   FR_hip   ----**/

//     a1->_body[0].set_mcI(a1->_urdf)


//     com << -0.003311, -0.000635, 3.1e-05;
//     Ic << 0.000469246, 9.409e-06, -3.42e-07,
//         9.409e-06, 0.00080749, 4.66e-07,
//         -3.42e-07, 4.66e-07, 0.000552929;
//     a1->_body[0].set_mcI(0.696, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0.1805, -0.047, 0;
//     a1->_joint[0].set_type(JointType::RX );
//     // a1->_joint[0].set_rpy_xyz(rpy, xyz);

//     /**-----   FR_thigh   ----**/
//     com << -0.003237, 0.022327, -0.027326;
//     Ic << 0.005529065, -4.825e-06, 0.000343869,
//         -4.825e-06, 0.005139339, -2.2448e-05,
//         0.000343869, -2.2448e-05, 0.001367788;
//     a1->_body[1].set_mcI(1.013, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, -0.0838, 0;
//     a1->_joint[1].set_type(JointType::RY );
//     a1->_joint[1].set_rpy_xyz(rpy, xyz);

//     /**-----   FR_calf   ----**/
//     com << 0.006435, 0.0, -0.107388;
//     Ic << 0.002997972, 0.0, -0.000141163,
//         0.0, 0.003014022, 0.0,
//         -0.000141163, 0.0, 3.2426e-05;
//     a1->_body[2].set_mcI(0.226, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     a1->_joint[2].set_type(JointType::RY );
//     a1->_joint[2].set_rpy_xyz(rpy, xyz);

//     /**-----   FL_hip   ----**/
//     com << -0.003311, 0.000635, 3.1e-05;
//     Ic << 0.000469246, -9.409e-06, -3.42e-07,
//         -9.409e-06, 0.00080749, -4.66e-07,
//         -3.42e-07, -4.66e-07, 0.000552929;
//     a1->_body[3].set_mcI(0.696, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0.1805, 0.047, 0;
//     a1->_joint[3].set_type(JointType::RX );
//     a1->_joint[3].set_rpy_xyz(rpy, xyz);

//     /**-----   FL_thigh   ----**/
//     com << -0.003237, -0.022327, -0.027326;
//     Ic << 0.005529065, 4.825e-06, 0.000343869,
//         4.825e-06, 0.005139339, 2.2448e-05,
//         0.000343869, 2.2448e-05, 0.001367788;
//     a1->_body[4].set_mcI(1.013, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0.0838, 0;
//     a1->_joint[4].set_type(JointType::RY );
//     a1->_joint[4].set_rpy_xyz(rpy, xyz);

//     /**-----   FL_calf   ----**/
//     com << 0.006435, 0.0, -0.107388;
//     Ic << 0.002997972, 0.0, -0.000141163,
//         0.0, 0.003014022, 0.0,
//         -0.000141163, 0.0, 3.2426e-05;
//     a1->_body[5].set_mcI(0.226, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     a1->_joint[5].set_type(JointType::RY );
//     a1->_joint[5].set_rpy_xyz(rpy, xyz);

//     /**-----   RR_hip   ----**/
//     com << 0.003311, -0.000635, 3.1e-05;
//     Ic << 0.000469246, -9.409e-06, 3.42e-07,
//         -9.409e-06, 0.00080749, 4.66e-07,
//         3.42e-07, 4.66e-07, 0.000552929;
//     a1->_body[6].set_mcI(0.696, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << -0.1805, -0.047, 0;
//     a1->_joint[6].set_type(JointType::RX );
//     a1->_joint[6].set_rpy_xyz(rpy, xyz);

//     /**-----   RR_thigh   ----**/
//     com << -0.003237, 0.022327, -0.027326;
//     Ic << 0.005529065, -4.825e-06, 0.000343869,
//         -4.825e-06, 0.005139339, -2.2448e-05,
//         0.000343869, -2.2448e-05, 0.001367788;
//     a1->_body[7].set_mcI(1.013, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, -0.0838, 0;
//     a1->_joint[7].set_type(JointType::RY );
//     a1->_joint[7].set_rpy_xyz(rpy, xyz);

//     /**-----   RR_calf   ----**/
//     com << 0.006435, 0.0, -0.107388;
//     Ic << 0.002997972, 0.0, -0.000141163,
//         0.0, 0.003014022, 0.0,
//         -0.000141163, 0.0, 3.2426e-05;
//     a1->_body[8].set_mcI(0.226, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     a1->_joint[8].set_type(JointType::RY );
//     a1->_joint[8].set_rpy_xyz(rpy, xyz);

//     /**-----   RL_hip   ----**/
//     com << 0.003311, 0.000635, 3.1e-05;
//     Ic << 0.000469246, 9.409e-06, 3.42e-07,
//         9.409e-06, 0.00080749, -4.66e-07,
//         3.42e-07, -4.66e-07, 0.000552929;
//     a1->_body[9].set_mcI(0.696, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << -0.1805, 0.047, 0;
//     a1->_joint[9].set_type(JointType::RX );
//     a1->_joint[9].set_rpy_xyz(rpy, xyz);

//     /**-----   RL_thigh   ----**/
//     com << -0.003237, -0.022327, -0.027326;
//     Ic << 0.005529065, 4.825e-06, 0.000343869,
//         4.825e-06, 0.005139339, 2.2448e-05,
//         0.000343869, 2.2448e-05, 0.001367788;
//     a1->_body[10].set_mcI(1.013, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0.0838, 0;
//     a1->_joint[10].set_type(JointType::RY );
//     a1->_joint[10].set_rpy_xyz(rpy, xyz);

//     /**-----   RL_calf   ----**/
//     com << 0.006435, 0.0, -0.107388;
//     Ic << 0.002997972, 0.0, -0.000141163,
//         0.0, 0.003014022, 0.0,
//         -0.000141163, 0.0, 3.2426e-05;
//     a1->_body[11].set_mcI(0.226, com, Ic);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     a1->_joint[11].set_type(JointType::RY );
//     a1->_joint[11].set_rpy_xyz(rpy, xyz);

   

//     for (int i = 0; i < a1->_NB; i++)
//     {
//         // for every joint
//         for (int j = 0; j < a1->_joint[j]._DOF; j++)
//         {
//             a1->_q[j] = a1->_joint[j]._Init_value[j];
//         }
//         Rp2T(a1->_joint[i]._rpyMat, a1->_joint[i]._xyz, a1->Tj[i]);
//         a1->Tq[i] = roz(a1->_q[i]);
//         a1->T_dwtree[i] = a1->Tj[i] * a1->Tq[i];
//     }

//     /*-- Loop joint --*/

//     // Setting loop joint 0
//     a1->_lpjoint[0]._pre = WORLD;
//     a1->_lpjoint[0]._suc = 2;
//     a1->_lpjoint[0].set_type(JointType::SPHERE, nullptr);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     Rp2T(rpy, xyz, a1->_lpjoint[0].Ts);
//     AdjointT(a1->_lpjoint[0].Ts, a1->_lpjoint[0].Xs);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, 0;
//     Rp2T(rpy, xyz, a1->_lpjoint[0].Tp);
//     AdjointT(a1->_lpjoint[0].Tp, a1->_lpjoint[0].Xp);

//     // Setting loop joint 1
//     a1->_lpjoint[1]._pre = WORLD;
//     a1->_lpjoint[1]._suc = 5;
//     a1->_lpjoint[1].set_type(JointType::SPHERE, nullptr);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     Rp2T(rpy, xyz, a1->_lpjoint[1].Ts);
//     AdjointT(a1->_lpjoint[1].Ts, a1->_lpjoint[1].Xs);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, 0;
//     Rp2T(rpy, xyz, a1->_lpjoint[1].Tp);
//     AdjointT(a1->_lpjoint[1].Tp, a1->_lpjoint[1].Xp);

//     // Setting loop joint 2
//     a1->_lpjoint[2]._pre = WORLD;
//     a1->_lpjoint[2]._suc = 8;
//     a1->_lpjoint[2].set_type(JointType::SPHERE, nullptr);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     Rp2T(rpy, xyz, a1->_lpjoint[2].Ts);
//     AdjointT(a1->_lpjoint[2].Ts, a1->_lpjoint[2].Xs);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, 0;
//     Rp2T(rpy, xyz, a1->_lpjoint[2].Tp);
//     AdjointT(a1->_lpjoint[2].Tp, a1->_lpjoint[2].Xp);

//     // Setting loop joint 3
//     a1->_lpjoint[3]._pre = WORLD;
//     a1->_lpjoint[3]._suc = 11;
//     a1->_lpjoint[3].set_type(JointType::SPHERE, nullptr);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, -0.2;
//     Rp2T(rpy, xyz, a1->_lpjoint[3].Ts);
//     AdjointT(a1->_lpjoint[3].Ts, a1->_lpjoint[3].Xs);
//     rpy << 1, 0, 0,
//         0, 1, 0,
//         0, 0, 1;
//     xyz << 0, 0, 0;
//     Rp2T(rpy, xyz, a1->_lpjoint[3].Tp);
//     AdjointT(a1->_lpjoint[3].Tp, a1->_lpjoint[3].Xp);

//     a1->Update_Model();

//     std::cout
//         << std::endl
//         << "Model building completed! NB = " << a1->_NB << " NL = " << a1->_NL << std::endl;
//     std::cout << "The parent set is [ ";
//     for (int i = 0; i < a1->_NB; i++)
//         std::cout << a1->_parent[i] << " ";
//     std::cout << "]" << std::endl;
// }