/*
 * Copyright 2014, Oscar E. Ramos Ponce, LAAS-CNRS
 *
 */

#ifndef __sot_torque_control_talos_common_H__
#define __sot_torque_control_talos_common_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(talos_common_EXPORTS)
#define TALOSCOMMON_EXPORT __declspec(dllexport)
#else
#define TALOSCOMMON_EXPORT __declspec(dllimport)
#endif
#else
#define TALOSCOMMON_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <initializer_list>
#include <map>

#include "boost/assign.hpp"
/* HELPER */
#include <dynamic-graph/signal-helper.h>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
namespace sot {
namespace torque_control {

#define N_JOINTS 32

#define RIGHT_FOOT_FRAME_NAME "RLEG_JOINT5"
#define LEFT_FOOT_FRAME_NAME "LLEG_JOINT5"

const double DEFAULT_MAX_DELTA_Q =
    0.1;  /// max joint position tracking error [rad]

const double DEFAULT_MAX_CURRENT =
    5;  /// max CURRENT (double in [0 Amp, 20 Amp])

// Information on the location of the IMU and F/T sensors of HRP-2
// copied from the urdf file (in stacks/hrp2/hrp2_14_description/urdf):
//      <joint name="AccelerometerJoint" type="fixed">
//        <origin xyz="-0.13 0.0 0.118" rpy="0.0 0.0 0.0"/>
//        <joint name="RightFootForceSensorJoint" type="fixed">
//          <origin xyz="0.0 0.0 -0.105" rpy="0.0 0.0 0.0"/>
//        <joint name="LeftFootForceSensorJoint" type="fixed">
//          <origin xyz="0.0 0.0 -0.105" rpy="0.0 0.0 0.0"/>
//        <joint name="RightHandForceSensorJoint" type="fixed">
//          <origin xyz="0.005 0.0 -0.05925" rpy="0.0 0.0 0.0"/>
//        <joint name="LeftHandForceSensorJoint" type="fixed">
//          <origin xyz="0.005 0.0 -0.05925" rpy="0.0 0.0 0.0"/>

/// Position of the IMU w.r.t. the frame of the hosting link (torso)
const double IMU_XYZ[3] = {-0.13, 0.0, 0.118};  // not updated for talos/talos

/// Position of the force/torque sensors w.r.t. the frame of the hosting link
const double RIGHT_FOOT_FORCE_SENSOR_XYZ[3] = {
    0.0, 0.0, -0.0};  // not updated for talos/talos
const double LEFT_FOOT_FORCE_SENSOR_XYZ[3] = {
    0.0, 0.0, -0.0};  // not updated for talos/talos
const double RIGHT_HAND_FORCE_SENSOR_XYZ[3] = {0.0, 0.0, -0.051};
const double LEFT_HAND_FORCE_SENSOR_XYZ[3] = {0.005, 0.0, -0.051};

/// Rotation angle around Z axis of the force/torque sensors w.r.t. the frame of
/// the hosting link
const double RIGHT_HAND_FORCE_SENSOR_Z_ROTATION =
    -0.5 * M_PI;  // not updated for talos/talos
const double LEFT_HAND_FORCE_SENSOR_Z_ROTATION =
    -0.5 * M_PI;  // not updated for talos/talos

/// Position of the foot soles w.r.t. the frame of the foot
const double RIGHT_FOOT_SOLE_XYZ[3] = {0.0, 0.0, -0.107};
const double LEFT_FOOT_SOLE_XYZ[3] = {0.0, 0.0, -0.107};

/// Position of the hand grippers w.r.t. the frame of the hand
const double RIGHT_HAND_GRIPPER_XYZ[3] = {0.0, 0.0, -0.02875};
const double LEFT_HAND_GRIPPER_XYZ[3] = {0.0, 0.0, 0.02025};

/// Percentage of mass of the link that is measured by the F/T sensors
const double RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE =
    0.65;  // not updated for talos/talos
const double LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE =
    0.65;  // not updated for talos/talos
const double RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE =
    0.75;  // not updated for talos/talos
const double LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE =
    0.75;  // not updated for talos/talos

struct JointLimits {
  double upper;
  double lower;

  JointLimits() : upper(0.0), lower(0.0) {}

  JointLimits(double l, double u) : upper(u), lower(l) {}
};

/// Map from joint names to joint ids
struct JointUtil {
  static std::map<unsigned int, JointLimits> create_id_2_limits_map() {
    std::map<unsigned int, JointLimits> m;
    m[0] = JointLimits(-0.349065850399, 1.57079632679);    // left leg 1
    m[1] = JointLimits(-0.5236, 0.5236);                   // left leg 2
    m[2] = JointLimits(-2.095, 0.7);                       // left leg 3
    m[3] = JointLimits(0.0, 2.618);                        // left leg 4
    m[4] = JointLimits(-1.309, 0.768);                     // left leg 5
    m[5] = JointLimits(-0.5236, 0.5236);                   // left leg 6
    m[6] = JointLimits(-1.57079632679, 0.349065850399);    // right leg 1
    m[7] = JointLimits(-0.5236, 0.5236);                   // right leg 2
    m[8] = JointLimits(-2.095, 0.7);                       // right leg 3
    m[9] = JointLimits(0.0, 2.618);                        // right leg 4
    m[10] = JointLimits(-1.309, 0.768);                    // right leg 5
    m[11] = JointLimits(-0.5236, 0.5236);                  // right leg 6
    m[12] = JointLimits(-1.308996939, 1.308996939);        // torso 1
    m[13] = JointLimits(-0.261799387799, 0.785398163397);  // torso 2
    m[14] = JointLimits(-1.57079632679, 0.523598775598);  // left arm 1 shoulder
    m[15] = JointLimits(0.0, 2.87979326579);              // left arm 2 shoulder
    m[16] = JointLimits(-2.44346095279, 2.44346095279);   // left arm 3 shoulder
    m[17] = JointLimits(-2.35619449019, 0.0);             // left arm 4 elbow
    m[18] = JointLimits(-2.53072741539, 2.53072741539);   // left arm 5 wrist
    m[19] = JointLimits(-1.3962634016, 1.3962634016);     // left arm 6 wrist
    m[20] = JointLimits(-0.698131700798, 0.698131700798);  // left arm 7 wrist
    m[21] = JointLimits(-1.0471975512, 0.0);               // left gripper
    m[22] =
        JointLimits(-0.523598775598, 1.57079632679);     // right arm 1 shoulder
    m[23] = JointLimits(-2.87979326579, 0.0);            // right arm 2 shoulder
    m[24] = JointLimits(-2.44346095279, 2.44346095279);  // right arm 3 shoulder
    m[25] = JointLimits(-2.35619449019, 0.0);            // right arm 4 elbow
    m[26] = JointLimits(-2.53072741539, 2.53072741539);  // right arm 5 wrist
    m[27] = JointLimits(-1.3962634016, 1.3962634016);    // right arm 6 wrist
    m[28] = JointLimits(-0.698131700798, 0.698131700798);  // right arm 7 wrist
    m[29] = JointLimits(-1.0471975512, 0.0);  //         right gripper
    m[30] = JointLimits(-0.261799387799, 0.785398163397);  // head 1
    m[31] = JointLimits(-1.308996939, 1.308996939);        // head 2
    return m;
  }

  static std::map<std::string, unsigned int> create_name_2_id_map() {
    std::map<std::string, unsigned int> m;
    m["lhy"] = 0;   // left hip yaw
    m["lhr"] = 1;   // left hip roll
    m["lhp"] = 2;   // left hip pitch
    m["lk"] = 3;    // left knee
    m["lap"] = 4;   // left ankle pitch
    m["lar"] = 5;   // left ankle roll
    m["rhy"] = 6;   // right hip yaw
    m["rhr"] = 7;   // right hip roll
    m["rhp"] = 8;   // right hip pitch
    m["rk"] = 9;    // right knee
    m["rap"] = 10;  // right ankle pitch
    m["rar"] = 11;  // right ankle roll
    m["ty"] = 12;   // torso yaw
    m["tp"] = 13;   // torso pitch
    m["lsy"] = 14;  // left shoulder yaw
    m["lsr"] = 15;  // left shoulder roll
    m["lay"] = 16;  // left arm yaw
    m["le"] = 17;   // left elbow
    m["lwy"] = 18;  // left wrist yaw
    m["lwp"] = 19;  // left wrist pitch
    m["lwr"] = 20;  // left wrist roll
    m["lh"] = 21;   // left hand
    m["rsy"] = 22;  // right shoulder yaw
    m["rsr"] = 23;  // right shoulder roll
    m["ray"] = 24;  // right arm yaw
    m["re"] = 25;   // right elbow
    m["rwy"] = 26;  // right wrist yaw
    m["rwp"] = 27;  // right wrist pitch
    m["rwr"] = 28;  // right wrist roll
    m["rh"] = 29;   // right hand
    m["hp"] = 30;   // head pitch
    m["hy"] = 31;   // head yaw
    return m;
  }

  static std::map<unsigned int, std::string> create_id_2_name_map(
      const std::map<std::string, unsigned int>& name_2_id_map) {
    std::map<unsigned int, std::string> m;
    std::map<std::string, unsigned int>::const_iterator it;
    for (it = name_2_id_map.begin(); it != name_2_id_map.end(); it++)
      m[it->second] = it->first;
    return m;
  }

  /** Given a joint name it finds the associated joint id.
   * If the specified joint name is not found it returns -1;
   * @param name Name of the joint to find.
   * @return The id of the specified joint, -1 if not found. */
  static int get_id_from_name(std::string name) {
    std::map<std::string, unsigned int>::const_iterator iter =
        name_2_id.find(name);
    if (iter == name_2_id.end()) return -1;
    return iter->second;
  }

  /** Given a joint id it finds the associated joint name.
   * If the specified joint is not found it returns "Joint name not found";
   * @param id Id of the joint to find.
   * @return The name of the specified joint, "Joint name not found" if not
   * found. */
  static std::string get_name_from_id(unsigned int id) {
    std::map<unsigned int, std::string>::const_iterator iter =
        id_2_name.find(id);
    if (iter == id_2_name.end()) return "Joint name not found";
    return iter->second;
  }

  /** Given a joint id it finds the associated joint limits.
   * If the specified joint is not found it returns JointLimits(0,0).
   * @param id Id of the joint to find.
   * @return The limits of the specified joint, JointLimits(0,0) if not found.
   */
  static JointLimits get_limits_from_id(unsigned int id) {
    std::map<unsigned int, JointLimits>::const_iterator iter =
        id_2_limits.find(id);
    if (iter == id_2_limits.end()) return JointLimits(0.0, 0.0);
    return iter->second;
  }

  static const std::map<std::string, unsigned int> name_2_id;
  static const std::map<unsigned int, std::string> id_2_name;
  static const std::map<unsigned int, JointLimits> id_2_limits;
};
const std::map<std::string, unsigned int> JointUtil::name_2_id =
    JointUtil::create_name_2_id_map();
const std::map<unsigned int, std::string> JointUtil::id_2_name =
    JointUtil::create_id_2_name_map(JointUtil::name_2_id);
const std::map<unsigned int, JointLimits> JointUtil::id_2_limits =
    JointUtil::create_id_2_limits_map();

struct ForceLimits {
  Eigen::VectorXd upper;
  Eigen::VectorXd lower;

  ForceLimits()
      : upper(dynamicgraph::sot::Vector6d::Zero()),
        lower(dynamicgraph::sot::Vector6d::Zero()) {}

  ForceLimits(const Eigen::VectorXd& l, const Eigen::VectorXd& u)
      : upper(u), lower(l) {}
};

enum ForceID {
  FORCE_ID_RIGHT_FOOT = 0,
  FORCE_ID_LEFT_FOOT = 1,
  FORCE_ID_RIGHT_HAND = 2,
  FORCE_ID_LEFT_HAND = 3
};

/// Map from force names to force ids
struct ForceUtil {
  static std::map<unsigned int, ForceLimits> create_id_2_limits_map() {
    dynamicgraph::sot::Vector6d fMax, fMin;
    fMax << 100.0, 100.0, 300.0, 80.0, 80.0, 30.0;
    fMin = -fMax;
    std::map<unsigned int, ForceLimits> m;
    m[0] = ForceLimits(fMin, fMax);  // right foot
    m[1] = ForceLimits(fMin, fMax);  // left foot
    m[2] = ForceLimits(fMin, fMax);  // right hand
    m[3] = ForceLimits(fMin, fMax);  // left hand
    return m;
  }

  static std::map<std::string, unsigned int> create_name_2_id_map() {
    std::map<std::string, unsigned int> m;
    m["rf"] = 0;  // right foot
    m["lf"] = 1;  // left foot
    m["rh"] = 2;  // right hand
    m["lh"] = 3;  // left hand
    return m;
  }

  static std::map<unsigned int, std::string> create_id_2_name_map(
      const std::map<std::string, unsigned int>& name_2_id_map) {
    std::map<unsigned int, std::string> m;
    std::map<std::string, unsigned int>::const_iterator it;
    for (it = name_2_id_map.begin(); it != name_2_id_map.end(); it++)
      m[it->second] = it->first;
    return m;
  }

  /** Given a force name it finds the associated id.
   * If the specified force name is not found it returns -1;
   * @param name Name of the force to find.
   * @return The id of the specified force, -1 if not found. */
  static int get_id_from_name(std::string name) {
    std::map<std::string, unsigned int>::const_iterator iter =
        name_2_id.find(name);
    if (iter == name_2_id.end()) return -1;
    return iter->second;
  }

  /** Given a force id it finds the associated name.
   * If the specified force is not found it returns "Force name not found";
   * @param id Id of the force to find.
   * @return The name of the specified force, "Force name not found" if not
   * found. */
  static std::string get_name_from_id(unsigned int id) {
    std::map<unsigned int, std::string>::const_iterator iter =
        id_2_name.find(id);
    if (iter == id_2_name.end()) return "Force name not found";
    return iter->second;
  }

  /** Given a force id it finds the associated limits.
   * If the specified force is not found it returns ForceLimits(0,0).
   * @param id Id of the force to find.
   * @return The limits of the specified force, ForceLimits(0,0) if not found.
   */
  static ForceLimits get_limits_from_id(unsigned int id) {
    std::map<unsigned int, ForceLimits>::const_iterator iter =
        id_2_limits.find(id);
    if (iter == id_2_limits.end()) return ForceLimits();
    return iter->second;
  }

  static const std::map<std::string, unsigned int> name_2_id;
  static const std::map<unsigned int, std::string> id_2_name;
  static const std::map<unsigned int, ForceLimits> id_2_limits;
};
const std::map<std::string, unsigned int> ForceUtil::name_2_id =
    ForceUtil::create_name_2_id_map();
const std::map<unsigned int, std::string> ForceUtil::id_2_name =
    ForceUtil::create_id_2_name_map(ForceUtil::name_2_id);
const std::map<unsigned int, ForceLimits> ForceUtil::id_2_limits =
    ForceUtil::create_id_2_limits_map();

bool base_se3_to_sot(dynamicgraph::sot::ConstRefVector pos,
                     dynamicgraph::sot::ConstRefMatrix R,
                     dynamicgraph::sot::RefVector q_sot);
bool base_urdf_to_sot(dynamicgraph::sot::ConstRefVector q_urdf,
                      dynamicgraph::sot::RefVector q_sot);
bool base_sot_to_urdf(dynamicgraph::sot::ConstRefVector q_sot,
                      dynamicgraph::sot::RefVector q_urdf);
bool config_urdf_to_sot(dynamicgraph::sot::ConstRefVector q_urdf,
                        dynamicgraph::sot::RefVector q_sot);
bool config_sot_to_urdf(dynamicgraph::sot::ConstRefVector q_sot,
                        dynamicgraph::sot::RefVector q_urdf);
bool velocity_urdf_to_sot(dynamicgraph::sot::ConstRefVector v_urdf,
                          dynamicgraph::sot::RefVector v_sot);
bool velocity_sot_to_urdf(dynamicgraph::sot::ConstRefVector v_sot,
                          dynamicgraph::sot::RefVector v_urdf);
bool joints_urdf_to_sot(dynamicgraph::sot::ConstRefVector q_urdf,
                        dynamicgraph::sot::RefVector q_sot);
bool joints_sot_to_urdf(dynamicgraph::sot::ConstRefVector q_sot,
                        dynamicgraph::sot::RefVector q_urdf);

}  // namespace torque_control
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_torque_control_talos_common_H__
