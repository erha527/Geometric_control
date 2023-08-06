
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author yty 
 */

#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      fail_detec_(false),
      ctrl_enable_(true),
      landing_commanded_(false),
      feedthrough_enable_(false),
      node_state(WAITING_FOR_HOME_POSE) {

/////////////////////////////////////////////////////

        e_R = Eigen::Vector3d::Zero();
        e_Angluar_v = Eigen::Vector3d::Zero();
        V_v = Eigen::Vector3d::Zero();
        V_v_true = Eigen::Vector3d::Zero();
        F_v = Eigen::Vector3d::Zero();

        f_d = Eigen::Vector3d::Zero();
        f_d_last = Eigen::Vector3d::Zero();
        b1d = Eigen::Vector3d::Zero();
        b1d_m = Eigen::Vector3d::Zero();
        b2d = Eigen::Vector3d::Zero();
        b3d = Eigen::Vector3d::Zero();
        R_d = Eigen::Matrix3d::Identity();
        R_d_time = Eigen::Matrix3d::Identity();
        Angluar_v_d = Eigen::Vector3d::Zero();
        Angluar_v_d_last = Eigen::Vector3d::Zero();
        Angluar_v_cur = Eigen::Vector3d::Zero();
        Acc_v_cur = Eigen::Vector3d::Zero();        
    
        st_d = Eigen::Vector3d::Zero();
        st = Eigen::Vector3d::Zero();
        m_star << 0,0,1;//m*
        axis_z << 0,0,1; // e_z

        R_base2frd << 1.0, 0.0, 0.0, 
                      0.0, -1.0, 0.0, 
                      0.0, 0.0, -1.0;
        J <<0.029125,  0,          0,
            0,          0.029125,  0,
            0,          0,          0.055225;
        R_YX = Eigen::Matrix3d::Identity();
        R_Z = Eigen::Matrix3d::Identity();
        R = Eigen::Matrix3d::Identity();
        
        angular_velocity.x=0; angular_velocity.y=0; angular_velocity.z=0; 
        nh_private_.param<double>("k_r0", k_r0, 0.4);
        nh_private_.param<double>("k_r1", k_r1, 0.4);
        nh_private_.param<double>("k_r2", k_r2, 0.2);
        nh_private_.param<double>("k_angluar_v0", k_angluar_v0, 0.06);
        nh_private_.param<double>("k_angluar_v1", k_angluar_v1, 0.06);
        nh_private_.param<double>("k_angluar_v2", k_angluar_v2, 0.02);
        nh_private_.param<double>("k_i0", k_i0, 0.06);
        nh_private_.param<double>("k_i1", k_i1, 0.06);
        nh_private_.param<double>("k_i2", k_i2, 0.02);
        g = 9.8091;
        m = 1.5;
        t = 0.1;
        c = 1.0; 
        e_yaw = 0.0;
        //控制输出量初始化
        T = m*g; //intilize is gravity
        torque = Eigen::Vector3d::Zero();
        //控制输入量初始化
      
        k_r <<  k_r0, 0.0, 0.0, 
                0.0, k_r1, 0.0, 
                0.0, 0.0, k_r2;//0.4
        k_i << k_i0, 0.0, 0.0, 
                0.0, k_i1, 0.0, 
                0.0, 0.0, k_i2;       
        k_angluar_v <<  k_angluar_v0, 0.0, 0.0, 
                        0.0, k_angluar_v1, 0.0, 
                        0.0, 0.0, k_angluar_v2;//0.08
    
     
        
/////////////////////////////////////////////
  
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());////arm
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  imu_sub = nh_.subscribe("/mavros/imu/data",1,&geometricCtrl::IMUUpdate,this,
                                ros::TransportHints().tcpNoDelay()); 


  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.008), &geometricCtrl::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this);  // Define timer for constant loop rate
  actuatorVelPub_= nh_.advertise<mavros_msgs::ActuatorControl>
        ("mavros/actuator_control",2);
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  

  testPub_ = nh_.advertise<geometry_msgs::PoseStamped>("attitude/test", 1);
  test_MOMENT_Pub_ = nh_.advertise<geometry_msgs::PoseStamped>("MOMENT_THIRD/test", 1);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.567);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);
  ///////////////////////////////////////////////////////////////////
  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}


void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  // reference_request_last_ = reference_request_now_;

  // targetPos_prev_ = targetPos_;
  // targetVel_prev_ = targetVel_;

  // reference_request_now_ = ros::Time::now();
  // reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  // targetPos_ = toEigen(msg.position);
  // targetVel_ = toEigen(msg.velocity);

  // if (msg.type_mask == 1) {
  //   targetAcc_ = toEigen(msg.acceleration);
  //   targetJerk_ = toEigen(msg.jerk);
  //   targetSnap_ = Eigen::Vector3d::Zero();

  // } else if (msg.type_mask == 2) {
  //   targetAcc_ = toEigen(msg.acceleration);
  //   targetJerk_ = Eigen::Vector3d::Zero();
  //   targetSnap_ = Eigen::Vector3d::Zero();

  // } else if (msg.type_mask == 4) {
  //   targetAcc_ = Eigen::Vector3d::Zero();
  //   targetJerk_ = Eigen::Vector3d::Zero();
  //   targetSnap_ = Eigen::Vector3d::Zero();

  // } else {
  //   targetAcc_ = toEigen(msg.acceleration);
  //   targetJerk_ = toEigen(msg.jerk);
  //   targetSnap_ = toEigen(msg.snap);
  // }
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  // trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  // reference_request_last_ = reference_request_now_;

  // targetPos_prev_ = targetPos_;
  // targetVel_prev_ = targetVel_;

  // reference_request_now_ = ros::Time::now();
  // reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  // targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  // targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  // targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  // targetJerk_ = Eigen::Vector3d::Zero();
  // targetSnap_ = Eigen::Vector3d::Zero();

  // if (!velocity_yaw_) {
  //   Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
  //                        pt.transforms[0].rotation.z);
  //   Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
  //   mavYaw_ = rpy(2);
  // }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  // mavAtt_(0) = msg.pose.orientation.w;
  // mavAtt_(1) = msg.pose.orientation.x;
  // mavAtt_(2) = msg.pose.orientation.y;
  // mavAtt_(3) = msg.pose.orientation.z;
  // R = quat2RotMatrix(mavAtt_);
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  if(IMUUpdateState)
        {
  mavVel_ = toEigen(msg.twist.linear);
  //mavRate_ = toEigen(msg.twist.angular);
  mavRate_(0)= msg.twist.angular.x;
  mavRate_(1)= -msg.twist.angular.y;
  mavRate_(2)= -msg.twist.angular.z;
        }
}



void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = MISSION_EXECUTION;
      break;

    case MISSION_EXECUTION: {
      
      if (feedthrough_enable_) {
        desired_acc = targetAcc_;
      } else {
        desired_acc = controlPosition(targetPos_,targetVel_,targetAcc_);
      }
     
      computeBodyRateCmd(cmdBodyRate_, desired_acc);

      pubReferencePose(targetPos_, q_des);
      testPub_.publish(test_error);
      test_MOMENT_Pub_.publish(test_moment_third);
      pubMomentCommands(cmdBodyRate_, q_des);
      //pubRateCommands(cmdBodyRate_, q_des);
      break;
    }
    
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}
void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 7;  // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubMomentCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &torque)
{
  mavros_msgs::ActuatorControl mix_u;  
  mix_u.group_mix = 0;
                mix_u.controls[0] = std::max(-0.3, std::min(0.3, -cmd(0)));
                mix_u.controls[1] = std::max(-0.3, std::min(0.3, cmd(1)));
                mix_u.controls[2] = std::max(-0.3, std::min(0.3, cmd(2)));
                mix_u.controls[3] = cmd(3);
               
                mix_u.controls[4] = 0.0;
                mix_u.controls[5] = 0.0; 
                mix_u.controls[6] = 0.0;
                mix_u.controls[7] = 0.0;
  actuatorVelPub_.publish(mix_u);
}

void geometricCtrl::IMUUpdate(const sensor_msgs::Imu &msg)
    {
        IMUUpdateState = true;
         sensor_msgs::Imu imu_data = msg; //frame_id = map 
        Acc_v_cur(0)  = imu_data.linear_acceleration.x;
        Acc_v_cur(1)  = imu_data.linear_acceleration.y;
        Acc_v_cur(2)  = imu_data.linear_acceleration.z; 
               
         angular_velocity = imu_data.angular_velocity;
        //  Angluar_v_cur(0) = angular_velocity.x;
        //  Angluar_v_cur(1) = -angular_velocity.y;
        //  Angluar_v_cur(2) = -angular_velocity.z;
         
         Angluar_v_cur(0) = angular_velocity.x;
         Angluar_v_cur(1) = angular_velocity.y;
         Angluar_v_cur(2) = angular_velocity.z;
        //  tf::quaternionMsgToTF(imu_data.orientation, quat);
        mavAtt_(0) = msg.orientation.w;
        mavAtt_(1) = msg.orientation.x;
        mavAtt_(2) = msg.orientation.y;
        mavAtt_(3) = msg.orientation.z;
        R = quat2RotMatrix(mavAtt_);
        // q_des = rot2Quaternion(R);
        //  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        //  roll = roll;
        //  pitch = pitch;
        //  yaw = yaw;

        //  Eigen::Matrix3d R_y = Eigen::Matrix3d::Zero();
        //  Eigen::Matrix3d R_x = Eigen::Matrix3d::Zero();
        //  Eigen::Matrix3d R_z = Eigen::Matrix3d::Zero();

        //  // 弧度  frame_id = base_link_ned //转到机体坐标系下
        //  R_y <<cos(pitch),0,sin(pitch),0,1,0,-sin(pitch),0,cos(pitch);
        //  R_x <<1,0,0,0,cos(roll),-sin(roll),0,sin(roll),cos(roll);
        //  R_z <<cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1;

        // R_YX = R_y * R_x;
        // R_Z = R_z;
        // R = R_Z * R_YX;

    }

Eigen::Vector3d geometricCtrl::vex_matrix(Eigen::Matrix3d S)
    {
        Eigen::Vector3d v;
        v << 0.5 * (S(2,1) - S(1,2)), 0.5 * (S(0,2) - S(2,0)), 0.5 * (S(1,0) - S(0,1));
        return v;        
    }
Eigen::Matrix3d geometricCtrl::matrix_vex(Eigen::Vector3d v)
    {
        Eigen::Matrix3d mat;
        mat << 0, -v(2), v(1), 
               v(2), 0, -v(0), 
               -v(1), v(0), 0;
        return mat;
    }

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

// Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos
//                                                ) {
//   /// Compute BodyRate commands using differential flatness
//   /// Controller based on Faessler 2017
//   Eigen::Vector3d  a_ref;
//   Eigen::Vector3d target_vel;
//   if (velocity_yaw_) {
//     mavYaw_ = getVelocityYaw(mavVel_);
//   }
  
//   const Eigen::Vector3d pos_error = mavPos_ - target_pos;
//   //  target_vel = 0.5*pos_error;
//   target_vel <<0.0 ,0.0, 0.0;
//    vel_error = mavVel_ - target_vel;
//   //a_ref = 0.2*vel_error;
//   //a_ref <<0.0 ,0.0, 0.0;
//   // Position Controller
//   const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

//   // Rotor Drag compensation
//   //const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag
  
//   // Reference acceleration
//   const Eigen::Vector3d a_des = a_fb + a_ref  - g_;
  
//   return a_des;
// }
Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
 const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

  return a_des;
}
void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  static Eigen::Vector3d f_d_time = Eigen::Vector3d::Zero();
  Eigen::Matrix3d one;one<<1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
  Eigen::Vector3d b1d_time,b3d_time,b2d_time,b1d_m_time,trans_vector1,trans_vector2,trans_vector3,trans_norm;   
  Eigen::Vector3d a(0.1, 0.1, 0.1);
  Eigen::Vector3d b(0.2, 0.2, 0.2);
  Eigen::VectorXd Fd_norm_dot,zbd_xcd_norm_dot;
        t_now = ros::Time::now();
        b1d_time = Eigen::Vector3d::Zero();
        f_d=one*a_des;
        //q_des = acc2quaternion(a_des,0);
        b1d << 1.0,0.0,0.0;
        b1d.normalize();
        b3d = f_d;//将fd转到自身坐标系下
        //f_d_time = one*(Kpos_.asDiagonal()*vel_error + Kvel_.asDiagonal()*Acc_v_cur + desired_acc);
        //Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;
        f_d_time =one*((Kpos_ - a - 0.5* Kvel_).asDiagonal()*mavVel_ + (b + Kvel_).asDiagonal()*Acc_v_cur);
        //f_d_time =one*((Kpos_ - a - 0.5* Kvel_).asDiagonal()*mavVel_);
        b3d = b3d/b3d.norm();
        //b3d.normalize();//归一化
        b2d = b3d.cross(b1d);//叉乘
        b2d = b2d/b2d.norm();
        //b2d.normalize();
        b1d_m = b2d.cross(b3d);
        // b1d_m.normalize();
        b1d_m = b1d_m/b1d_m.norm();
        R_d(0,0) = b1d_m(0); R_d(0,1) = b2d(0); R_d(0,2) = b3d(0);
        R_d(1,0) = b1d_m(1); R_d(1,1) = b2d(1); R_d(1,2) = b3d(1);
        R_d(2,0) = b1d_m(2); R_d(2,1) = b2d(2); R_d(2,2) = b3d(2);
        q_des = rot2Quaternion(R_d);
        
        //b1d 导数
        // trans_vector4 << -sin(yaw)*Angluar_v_cur[2],cos(yaw)*Angluar_v_cur[2],0;//b1c导数.
        
        // b1d_time = (I_3_3 +matrix_vex(zw)*sin(w)+ matrix_vex(zw)*matrix_vex(zw)*(1-cos(w)))*trans_vector4;
        // b1d_time = b1d_time + (matrix_vex(zw)*cos(w)*k_yaw*k_yaw*t*(1-2*sin(yaw/2)*sin(yaw/2))*e_yaw+matrix_vex(zw)*matrix_vex(zw)*sin(w)*k_yaw*t*k_yaw*(1-2*sin(yaw/2)*sin(yaw/2))*e_yaw)*b1c;
        // // b1d_time << 0,0,0;

        // b1d_time = (b1d - trans_vector4)/t;
///////////////////////////////////////////////////beifen 
        // trans_vector4 = b1d;
        // //b3d 导数
        // trans_vector1=(f_d-f_d_last)/t;
        // trans_vector1 = trans_vector1/f_d.norm();
        // b3d_time = b3d.cross(trans_vector1);
        // b3d_time= b3d_time.cross(b3d);

        // f_d_last = f_d;
        // //b2d导数
        // trans_vector2 = b3d_time.cross(b1d)+b3d.cross(b1d_time);
        // trans_norm = b3d.cross(b1d);

        // trans_vector2 = trans_vector2/trans_norm.norm();

        // b2d_time = b2d.cross(trans_vector2);
        // b2d_time = b2d_time.cross(b2d);
        // //b1d_m
        // b1d_m_time = b2d_time.cross(b3d) + b2d.cross(b3d_time);
////////////////////////////////////////////////////////////////////////
        // b1d_time << 0,0,0;
        // Fd_norm_dot = f_d.transpose()*f_d_time/f_d.norm();
        // b3d_time = (f_d_time*f_d.norm() - f_d*Fd_norm_dot)/(f_d.norm()*f_d.norm());
        // //b3d_time = (f_d_time*f_d.norm())/(f_d.norm()*f_d.norm());
        // trans_vector2 = b3d_time.cross(b1d)+b3d.cross(b1d_time);
        // zbd_xcd_norm_dot = (b3d.cross(b1d)).transpose() * trans_vector2/(b3d.cross(b1d)).norm();
        // b2d_time = (trans_vector2*((b3d.cross(b1d)).norm()) - b3d.cross(b1d)*zbd_xcd_norm_dot)/(((b3d.cross(b1d)).norm())*(b3d.cross(b1d)).norm());
        // b1d_m_time = b2d_time.cross(b3d) + b2d.cross(b3d_time);
      //   R_d_time(0,0) = b1d_m_time(0); R_d_time(0,1) = b2d_time(0); R_d_time(0,2) = b3d_time(0);
      //   R_d_time(1,0) = b1d_m_time(1); R_d_time(1,1) = b2d_time(1); R_d_time(1,2) = b3d_time(1);
      //   R_d_time(2,0) = b1d_m_time(2); R_d_time(2,1) = b2d_time(2); R_d_time(2,2) = b3d_time(2);
      //  std::cout << "rd\n" << R_d << std::endl;
      //  std::cout << "-v + w - v =\n" << R_d.transpose()*(R_d_time) << std::endl;
      //Angluar_v_d = vex_matrix(R_d.transpose()*(R_d_time));
std::cout << "rd\n" << R_d << std::endl;
////////////https://github.com/yorgoon/minimum-snap-geometric-control/blob/master/controller.m/////
//////////////////////////////////////https://www.cnblogs.com/QiQi-Robotics/p/14562475.html/////////////////////////////////////////////////////////////
        
        
        delta_t = (t_now - t_last).toSec();
        Angluar_v_d = vex_matrix((R_d*R_d_last.inverse()-one)/delta_t);
        
        std::cout << "rd_last\n" << R_d_last << std::endl;
        std::cout << "ertt\n" << R_d*R_d_last.inverse() << std::endl;
        std::cout << "Angluar_v_d_matrix\n" << (R_d*R_d_last.inverse()-one)/delta_t << std::endl;
        R_d_last = R_d;
        t_last = t_now;
///////////////////////////////////////////////////////////////////////////////////////////////////
      bodyrate_cmd = geometric_attcontroller(R, a_des, R_d);  // Calculate BodyRate   
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;///////////235ye

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

Eigen::Vector4d geometricCtrl::geometric_attcontroller(const Eigen::Matrix3d &rotmat, const Eigen::Vector3d &ref_acc,
                                                       const Eigen::Matrix3d &rotmat_d) {
  
  Eigen::Vector4d ratecmd;
                   // Rotation matrix of current attitude
  Eigen::Matrix3d rotmatQ;  // Rotation matrix of desired attitude
  Eigen::Vector3d error_att,mm;
  Eigen::Vector3d p1,p2,p3,p4,p5;
 

  now_ei_dt=ros::Time::now();
  ei_dt= (now_ei_dt - last_ei_dt).toSec();
  last_ei_dt=now_ei_dt;
  
  // e_Angluar_v = mavRate_;

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  e_Angluar_v =  Angluar_v_cur - rotmat.transpose()*rotmat_d*Angluar_v_d;
  //e_Angluar_v = Angluar_v_cur;
  //std::cout << "er\n" << rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d << std::endl;
  e_R+=(error_att+e_R_last)*ei_dt/2;
  e_R_last=error_att;
  p4 = -k_i*e_R;
  mm=mavRate_.cross(J*mavRate_);
  p1 = matrix_vex(Angluar_v_cur)*J*Angluar_v_cur;
  p2 = - k_angluar_v*e_Angluar_v;
  p3 = - k_r*error_att; 
  torque =p2+p3+p1; 
  
  //torque = p3;
  rotmatQ = quat2RotMatrix(mavAtt_);
  const Eigen::Vector3d zb = rotmat.col(2);Eigen::Matrix3d one;one<<1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
  ratecmd(3) =
     std::max(0.0, std::min(1.0, norm_thrust_const_ * (one*ref_acc).dot(zb) + norm_thrust_offset_)); 
  
      // Calculate thrust
  ratecmd(0) = torque(0);
  ratecmd(1) = torque(1);
  ratecmd(2) = torque(2); 
  //ROS_INFO("error_att=%f,%f,%f,torque=%f,%f,%f",error_att(0),error_att(1),error_att(2),torque(0),torque(1),torque(2));
  test_error.pose.orientation.w=error_att(0);
  test_error.pose.orientation.x=error_att(1);
  test_error.pose.orientation.y=error_att(2);
  test_error.pose.position.x=e_Angluar_v(0);
  test_error.pose.position.y=e_Angluar_v(1);
  test_error.pose.position.z=e_Angluar_v(2);
  test_moment_third.pose.position.x=torque(0);
  test_moment_third.pose.position.y=torque(1);
  test_moment_third.pose.position.z=torque(2);
  return ratecmd;
}


