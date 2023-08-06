/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <controller_msgs/FlatTarget.h>
#include <dynamic_reconfigure/server.h>

#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include "geometric_controller/common.h"

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class geometricCtrl {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber referenceSub_;
  ros::Subscriber flatreferenceSub_;
  ros::Subscriber multiDOFJointSub_;
  ros::Subscriber mavstateSub_;
  ros::Subscriber mavposeSub_, gzmavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::Subscriber yawreferenceSub_;
  ros::Publisher rotorVelPub_,  target_pose_pub_,angularVelPub_;
  ros::Publisher referencePosePub_;
  ros::Publisher posehistoryPub_;
  ros::Publisher systemstatusPub_,testPub_,test_MOMENT_Pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer ctrltriggerServ_;
  ros::ServiceServer land_service_;
  ros::Timer cmdloop_timer_, statusloop_timer_,canshutimer;
  ros::Time last_request_, reference_request_now_, reference_request_last_;
  ros::Time last_ei_dt,now_ei_dt,t_last,t_now;
  string mav_name_;
  bool fail_detec_, ctrl_enable_, feedthrough_enable_;
  int ctrl_mode_;
  bool landing_commanded_;
  bool sim_enable_;
  bool velocity_yaw_;
  double kp_rot_, kd_rot_;
  double reference_request_dt_,ei_dt,delta_t;
  double attctrl_tau_;
  double norm_thrust_const_, norm_thrust_offset_;
  double max_fb_acc_;
  double dx_, dy_, dz_;
  Eigen::Vector3d desired_acc;
  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  Eigen::Vector3d last_ref_acc_{Eigen::Vector3d::Zero()};
  double mavYaw_;
  Eigen::Vector3d g_,jerk_ref;
  Eigen::Vector4d mavAtt_, q_des;
  Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_, Kvel_, D_;
  Eigen::Vector3d a0, a1, tau;
  Eigen::Vector3d vel_error;
  double tau_x, tau_y, tau_z;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_,kA,kB,kC;
  int posehistory_window_;
  int flag=0;
  void pubMotorCommands();
  
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  
  void pubSystemStatus();
  
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  void keyboardCallback(const geometry_msgs::Twist &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void statusloopCallback(const ros::TimerEvent &event);
  
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);
  // Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos
  //                                 );
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc);
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
  Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                Eigen::Vector4d &curr_att);
  Eigen::Vector4d geometric_attcontroller(const Eigen::Matrix3d &rotmat_d, const Eigen::Vector3d &ref_acc,
                                                       const Eigen::Matrix3d &rotmat);
  Eigen::Vector4d jerkcontroller(const Eigen::Vector3d &ref_jerk, const Eigen::Vector3d &ref_acc,
                                 Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att);
  void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
  enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;

  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };
  geometry_msgs::Pose home_pose_;
  bool received_home_pose;
  /////////////////////////////////////////geometry_control////////
    ros::Subscriber  imu_sub;
    ros::Publisher actuatorVelPub_;
    void IMUUpdate(const sensor_msgs::Imu &msg);
    void pubMomentCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &torque);

    void R_dUpdate();
    void Angluar_xUpdate();
    void TorqueUpdate();
    Eigen::Vector3d vex_matrix(Eigen::Matrix3d S);
    Eigen::Matrix3d matrix_vex(Eigen::Vector3d v);
    
    Eigen::Matrix3d R_YX,R_Z,R,R_YXD,R_ZD,RD;
    Eigen::Matrix3d R_base2frd;//变换矩阵 将base转变为机体坐标系的

    Eigen::Vector3d V_v,V_v_true; // V_v定义的虚拟像平面速度   V_v_true真实值
    Eigen::Vector3d m_star;//（0，0，1）

    Eigen::Vector3d e_R,e_Angluar_v,e_R_last;//定义的几何误差
    Eigen::Vector3d axis_z;//z轴（0，0，1)
    Eigen::Vector3d F_v;
    Eigen::Vector3d f_d,f_d_last;
    Eigen::Vector3d b1d,b1d_m,b2d,b3d;
    Eigen::Matrix3d R_d,R_d_last;//期望的姿态
    Eigen::Matrix3d R_d_time;//期望的姿态导数
    Eigen::Vector3d Angluar_v_d;//期望的角速度
    Eigen::Vector3d Angluar_v_d_last;//上次期望的角速度
    Eigen::Vector3d Angluar_v_cur;//当前的角速度
    Eigen::Vector3d Acc_v_cur;//当前的加速度
    Eigen::Vector3d trans_vector4;
    Eigen::Matrix3d J;//惯性矩阵  

    geometry_msgs::Vector3 angular_velocity;//角速度 机体ned
    tf::Quaternion quat,quat_d;//当前姿态
    double roll,pitch,yaw,e_y,roll_d,pitch_d,yaw_d; // 当前弧度 phi,theta,Phi
    bool IMUUpdateState;
    Eigen::Vector3d b1c;//相机朝向
    Eigen::Vector3d zw;//0，0，1
    double w;//偏航控制量

    Eigen::Vector3d st_d,st;//
    
   
    Eigen::Vector3d  V_d,acc_d;
    double g;//重力加速度
    double m;//质量
    double t;//t=1.0/250=0.004时间间隔 1/t  为频率 250hz
    double c;
    double e_yaw;//偏航角角速度
    //控制输出量
    double T;
    Eigen::Vector3d torque;//输出力矩
    //控制输入量
    Eigen::Matrix3d k_1,k_2,k_i,k_r,k_angluar_v,k_F;
    double k_yaw;
    double k_angluar_v0,k_angluar_v1,k_angluar_v2,k_r0,k_r1,k_r2,k_i0,k_i1,k_i2;
    geometry_msgs::PoseStamped test_error,test_moment_third;
///////////////////////////////////////////////////////////////////

 public:
  
  geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~geometricCtrl();
  void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
    pos = mavPos_;
    att = mavAtt_;
    vel = mavVel_;
    angvel = mavRate_;
  };
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
    pos = mavPos_ - targetPos_;
    vel = mavVel_ - targetVel_;
  };
  
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };
  void setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; };
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif