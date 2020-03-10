/*
 * Copyright 2014, Oscar E. Ramos Ponce, LAAS-CNRS
 *
 */

#include <sot/torque-control/talos-common.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

namespace dynamicgraph {
namespace sot {
namespace torque_control {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

bool base_se3_to_sot(dg::sot::ConstRefVector pos, dg::sot::ConstRefMatrix R, dg::sot::RefVector q_sot) {
  assert(q_sot.size() == 6);
  assert(pos.size() == 3);
  assert(R.rows() == 3);
  assert(R.cols() == 3);
  // ********* Quat to RPY *********
  double r, p, y, m;
  m = sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));
  p = atan2(-R(2, 0), m);
  if (abs(abs(p) - M_PI / 2) < 0.001) {
    r = 0.0;
    y = -atan2(R(0, 1), R(1, 1));
  } else {
    y = atan2(R(1, 0), R(0, 0));
    r = atan2(R(2, 1), R(2, 2));
  }
  // *********************************
  q_sot[0] = pos[0];
  q_sot[1] = pos[1];
  q_sot[2] = pos[2];
  q_sot[3] = r;
  q_sot[4] = p;
  q_sot[5] = y;
  return true;
}

bool base_urdf_to_sot(dg::sot::ConstRefVector q_urdf, dg::sot::RefVector q_sot) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);
  // ********* Quat to RPY *********
  const double W = q_urdf[6];
  const double X = q_urdf[3];
  const double Y = q_urdf[4];
  const double Z = q_urdf[5];
  const Eigen::Matrix3d R = Eigen::Quaterniond(W, X, Y, Z).toRotationMatrix();
  return base_se3_to_sot(q_urdf.head<3>(), R, q_sot);
}

bool base_sot_to_urdf(dg::sot::ConstRefVector q_sot, dg::sot::RefVector q_urdf) {
  assert(q_urdf.size() == 7);
  assert(q_sot.size() == 6);
  // *********  RPY to Quat *********
  const double r = q_sot[3];
  const double p = q_sot[4];
  const double y = q_sot[5];
  const Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
  const Eigen::Quaternion<double> quat = yawAngle * pitchAngle * rollAngle;

  q_urdf[0] = q_sot[0];  // BASE
  q_urdf[1] = q_sot[1];
  q_urdf[2] = q_sot[2];
  q_urdf[3] = quat.x();
  q_urdf[4] = quat.y();
  q_urdf[5] = quat.z();
  q_urdf[6] = quat.w();

  return true;
}

bool config_urdf_to_sot(dg::sot::ConstRefVector q_urdf, dg::sot::RefVector q_sot) {
  assert(q_urdf.size() == N_JOINTS + 7);
  assert(q_sot.size() == N_JOINTS + 6);
  // ********* Quat to RPY *********
  //        const double W = q_urdf[6];
  //        const double X = q_urdf[3];
  //        const double Y = q_urdf[4];
  //        const double Z = q_urdf[5];
  //        const Eigen::Matrix3d M = Eigen::Quaterniond(W, X, Y, Z).toRotationMatrix();
  //        double r,p,y,m;
  //        m = sqrt(M(2, 1) *M(2, 1) + M(2, 2) * M(2, 2));
  //        p = atan2(-M(2, 0), m);
  //        if (abs(abs(p) - M_PI / 2) < 0.001 )
  //        {
  //          r = 0;
  //          y = -atan2(M(0, 1), M(1, 1));
  //        }
  //        else
  //        {
  //          y = atan2(M(1, 0), M(0, 0)) ;
  //          r = atan2(M(2, 1), M(2, 2)) ;
  //        }
  //        // *********************************
  //        q_sot[0 ]=q_urdf[0 ]; //BASE
  //        q_sot[1 ]=q_urdf[1 ];
  //        q_sot[2 ]=q_urdf[2 ];
  //        q_sot[3 ]=r;
  //        q_sot[4 ]=p;
  //        q_sot[5 ]=y;
  base_urdf_to_sot(q_urdf.head<7>(), q_sot.head<6>());
  joints_urdf_to_sot(q_urdf.tail<N_JOINTS>(), q_sot.tail<N_JOINTS>());
  //        q_sot[18]=q_urdf[7 ]; //HEAD
  //        q_sot[19]=q_urdf[8 ];

  //        q_sot[20]=q_urdf[9 ]; //CHEST
  //        q_sot[21]=q_urdf[10];

  //        q_sot[29]=q_urdf[11]; //LARM
  //        q_sot[30]=q_urdf[12];
  //        q_sot[31]=q_urdf[13];
  //        q_sot[32]=q_urdf[14];
  //        q_sot[33]=q_urdf[15];
  //        q_sot[34]=q_urdf[16];
  //        q_sot[35]=q_urdf[17];

  //        q_sot[22]=q_urdf[18]; //RARM
  //        q_sot[23]=q_urdf[19];
  //        q_sot[24]=q_urdf[20];
  //        q_sot[25]=q_urdf[21];
  //        q_sot[26]=q_urdf[22];
  //        q_sot[27]=q_urdf[23];
  //        q_sot[28]=q_urdf[24];

  //        q_sot[12]=q_urdf[25]; //LLEG
  //        q_sot[13]=q_urdf[26];
  //        q_sot[14]=q_urdf[27];
  //        q_sot[15]=q_urdf[28];
  //        q_sot[16]=q_urdf[29];
  //        q_sot[17]=q_urdf[30];

  //        q_sot[6 ]=q_urdf[31]; //RLEG
  //        q_sot[7 ]=q_urdf[32];
  //        q_sot[8 ]=q_urdf[33];
  //        q_sot[9 ]=q_urdf[34];
  //        q_sot[10]=q_urdf[35];
  //        q_sot[11]=q_urdf[36];
  return true;
}

bool config_sot_to_urdf(dg::sot::ConstRefVector q_sot, dg::sot::RefVector q_urdf) {
  assert(q_urdf.size() == N_JOINTS + 7);
  assert(q_sot.size() == N_JOINTS + 6);
  // *********  RPY to Quat *********
  //        const double r = q_sot[3];
  //        const double p = q_sot[4];
  //        const double y = q_sot[5];
  //        const Eigen::AngleAxisd  rollAngle(r, Eigen::Vector3d::UnitX());
  //        const Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
  //        const Eigen::AngleAxisd   yawAngle(y, Eigen::Vector3d::UnitZ());
  //        const Eigen::Quaternion<double> quat = yawAngle * pitchAngle * rollAngle;

  //        q_urdf[0 ]=q_sot[0 ]; //BASE
  //        q_urdf[1 ]=q_sot[1 ];
  //        q_urdf[2 ]=q_sot[2 ];
  //        q_urdf[3 ]=quat.x();
  //        q_urdf[4 ]=quat.y();
  //        q_urdf[5 ]=quat.z();
  //        q_urdf[6 ]=quat.w();

  base_sot_to_urdf(q_sot.head<6>(), q_urdf.head<7>());
  joints_sot_to_urdf(q_sot.tail<N_JOINTS>(), q_urdf.tail<N_JOINTS>());

  //        q_urdf[7 ]=q_sot[18]; //HEAD
  //        q_urdf[8 ]=q_sot[19];

  //        q_urdf[9 ]=q_sot[20]; //CHEST
  //        q_urdf[10]=q_sot[21];

  //        q_urdf[11]=q_sot[29]; //LARM
  //        q_urdf[12]=q_sot[30];
  //        q_urdf[13]=q_sot[31];
  //        q_urdf[14]=q_sot[32];
  //        q_urdf[15]=q_sot[33];
  //        q_urdf[16]=q_sot[34];
  //        q_urdf[17]=q_sot[35];

  //        q_urdf[18]=q_sot[22]; //RARM
  //        q_urdf[19]=q_sot[23];
  //        q_urdf[20]=q_sot[24];
  //        q_urdf[21]=q_sot[25];
  //        q_urdf[22]=q_sot[26];
  //        q_urdf[23]=q_sot[27];
  //        q_urdf[24]=q_sot[28];

  //        q_urdf[25]=q_sot[12]; //LLEG
  //        q_urdf[26]=q_sot[13];
  //        q_urdf[27]=q_sot[14];
  //        q_urdf[28]=q_sot[15];
  //        q_urdf[29]=q_sot[16];
  //        q_urdf[30]=q_sot[17];

  //        q_urdf[31]=q_sot[6 ]; //RLEG
  //        q_urdf[32]=q_sot[7 ];
  //        q_urdf[33]=q_sot[8 ];
  //        q_urdf[34]=q_sot[9 ];
  //        q_urdf[35]=q_sot[10];
  //        q_urdf[36]=q_sot[11];
  return true;
}

bool velocity_urdf_to_sot(dg::sot::ConstRefVector v_urdf, dg::sot::RefVector v_sot) {
  assert(v_urdf.size() == N_JOINTS + 6);
  assert(v_sot.size() == N_JOINTS + 6);
  v_sot.head<6>() = v_urdf.head<6>();
  joints_urdf_to_sot(v_urdf.tail<N_JOINTS>(), v_sot.tail<N_JOINTS>());

  //        v_sot[0 ]=v_urdf[0 ]; //BASE
  //        v_sot[1 ]=v_urdf[1 ];
  //        v_sot[2 ]=v_urdf[2 ];
  //        v_sot[3 ]=v_urdf[3 ];
  //        v_sot[4 ]=v_urdf[4 ];
  //        v_sot[5 ]=v_urdf[5 ];

  //        v_sot[18]=v_urdf[6 ]; //HEAD
  //        v_sot[19]=v_urdf[7 ];

  //        v_sot[20]=v_urdf[8 ]; //CHEST
  //        v_sot[21]=v_urdf[9];

  //        v_sot[29]=v_urdf[10]; //LARM
  //        v_sot[30]=v_urdf[11];
  //        v_sot[31]=v_urdf[12];
  //        v_sot[32]=v_urdf[13];
  //        v_sot[33]=v_urdf[14];
  //        v_sot[34]=v_urdf[15];
  //        v_sot[35]=v_urdf[16];

  //        v_sot[22]=v_urdf[17]; //RARM
  //        v_sot[23]=v_urdf[18];
  //        v_sot[24]=v_urdf[19];
  //        v_sot[25]=v_urdf[20];
  //        v_sot[26]=v_urdf[21];
  //        v_sot[27]=v_urdf[22];
  //        v_sot[28]=v_urdf[23];

  //        v_sot[12]=v_urdf[24]; //LLEG
  //        v_sot[13]=v_urdf[25];
  //        v_sot[14]=v_urdf[26];
  //        v_sot[15]=v_urdf[27];
  //        v_sot[16]=v_urdf[28];
  //        v_sot[17]=v_urdf[29];

  //        v_sot[6 ]=v_urdf[30]; //RLEG
  //        v_sot[7 ]=v_urdf[31];
  //        v_sot[8 ]=v_urdf[32];
  //        v_sot[9 ]=v_urdf[33];
  //        v_sot[10]=v_urdf[34];
  //        v_sot[11]=v_urdf[35];
  return true;
}

bool velocity_sot_to_urdf(dg::sot::ConstRefVector v_sot, dg::sot::RefVector v_urdf) {
  assert(v_urdf.size() == N_JOINTS + 6);
  assert(v_sot.size() == N_JOINTS + 6);
  v_urdf.head<6>() = v_sot.head<6>();
  joints_sot_to_urdf(v_sot.tail<N_JOINTS>(), v_urdf.tail<N_JOINTS>());

  //        v_urdf[0 ]=v_sot[0 ]; //BASE
  //        v_urdf[1 ]=v_sot[1 ];
  //        v_urdf[2 ]=v_sot[2 ];
  //        v_urdf[3 ]=v_sot[3 ];
  //        v_urdf[4 ]=v_sot[4 ];
  //        v_urdf[5 ]=v_sot[5 ];

  //        v_urdf[6 ]=v_sot[18]; //HEAD
  //        v_urdf[7 ]=v_sot[19];

  //        v_urdf[8 ]=v_sot[20]; //CHEST
  //        v_urdf[9 ]=v_sot[21];

  //        v_urdf[10]=v_sot[29]; //LARM
  //        v_urdf[11]=v_sot[30];
  //        v_urdf[12]=v_sot[31];
  //        v_urdf[13]=v_sot[32];
  //        v_urdf[14]=v_sot[33];
  //        v_urdf[15]=v_sot[34];
  //        v_urdf[16]=v_sot[35];

  //        v_urdf[17]=v_sot[22]; //RARM
  //        v_urdf[18]=v_sot[23];
  //        v_urdf[19]=v_sot[24];
  //        v_urdf[20]=v_sot[25];
  //        v_urdf[21]=v_sot[26];
  //        v_urdf[22]=v_sot[27];
  //        v_urdf[23]=v_sot[28];

  //        v_urdf[24]=v_sot[12]; //LLEG
  //        v_urdf[25]=v_sot[13];
  //        v_urdf[26]=v_sot[14];
  //        v_urdf[27]=v_sot[15];
  //        v_urdf[28]=v_sot[16];
  //        v_urdf[29]=v_sot[17];

  //        v_urdf[30]=v_sot[6 ]; //RLEG
  //        v_urdf[31]=v_sot[7 ];
  //        v_urdf[32]=v_sot[8 ];
  //        v_urdf[33]=v_sot[9 ];
  //        v_urdf[34]=v_sot[10];
  //        v_urdf[35]=v_sot[11];
  return true;
}

bool joints_urdf_to_sot(dg::sot::ConstRefVector q_urdf, dg::sot::RefVector q_sot) {
  assert(q_urdf.size() == N_JOINTS);
  assert(q_sot.size() == N_JOINTS);

  q_sot[12] = q_urdf[0];  // HEAD
  q_sot[13] = q_urdf[1];

  q_sot[14] = q_urdf[2];  // CHEST
  q_sot[15] = q_urdf[3];

  q_sot[23] = q_urdf[4];  // LARM
  q_sot[24] = q_urdf[5];
  q_sot[25] = q_urdf[6];
  q_sot[26] = q_urdf[7];
  q_sot[27] = q_urdf[8];
  q_sot[28] = q_urdf[9];
  q_sot[29] = q_urdf[10];

  q_sot[16] = q_urdf[11];  // RARM
  q_sot[17] = q_urdf[12];
  q_sot[18] = q_urdf[13];
  q_sot[19] = q_urdf[14];
  q_sot[20] = q_urdf[15];
  q_sot[21] = q_urdf[16];
  q_sot[22] = q_urdf[17];

  q_sot[6] = q_urdf[18];  // LLEG
  q_sot[7] = q_urdf[19];
  q_sot[8] = q_urdf[20];
  q_sot[9] = q_urdf[21];
  q_sot[10] = q_urdf[22];
  q_sot[11] = q_urdf[23];

  q_sot[0] = q_urdf[24];  // RLEG
  q_sot[1] = q_urdf[25];
  q_sot[2] = q_urdf[26];
  q_sot[3] = q_urdf[27];
  q_sot[4] = q_urdf[28];
  q_sot[5] = q_urdf[29];
  return true;
}

bool joints_sot_to_urdf(dg::sot::ConstRefVector q_sot, dg::sot::RefVector q_urdf) {
  assert(q_urdf.size() == N_JOINTS);
  assert(q_sot.size() == N_JOINTS);

  q_urdf[0] = q_sot[12];  // HEAD
  q_urdf[1] = q_sot[13];

  q_urdf[2] = q_sot[24];  // CHEST
  q_urdf[3] = q_sot[25];

  q_urdf[4] = q_sot[23];  // LARM
  q_urdf[5] = q_sot[24];
  q_urdf[6] = q_sot[25];
  q_urdf[7] = q_sot[26];
  q_urdf[8] = q_sot[27];
  q_urdf[9] = q_sot[28];
  q_urdf[10] = q_sot[29];

  q_urdf[11] = q_sot[16];  // RARM
  q_urdf[12] = q_sot[17];
  q_urdf[13] = q_sot[18];
  q_urdf[14] = q_sot[19];
  q_urdf[15] = q_sot[20];
  q_urdf[16] = q_sot[21];
  q_urdf[17] = q_sot[22];

  q_urdf[18] = q_sot[6];  // LLEG
  q_urdf[19] = q_sot[7];
  q_urdf[20] = q_sot[8];
  q_urdf[21] = q_sot[9];
  q_urdf[22] = q_sot[10];
  q_urdf[23] = q_sot[11];

  q_urdf[24] = q_sot[0];  // RLEG
  q_urdf[25] = q_sot[1];
  q_urdf[26] = q_sot[2];
  q_urdf[27] = q_sot[3];
  q_urdf[28] = q_sot[4];
  q_urdf[29] = q_sot[5];
  return true;
}

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph
