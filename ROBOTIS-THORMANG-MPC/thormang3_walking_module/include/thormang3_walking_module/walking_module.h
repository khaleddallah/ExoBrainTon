/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * online_walking_module.h
 *
 *  Created on: 2016. 9. 30.
 *      Author: Jay Song
 */

#ifndef THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_
#define THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>

#include "thormang3_walking_module/thormang3_online_walking.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/StartWalking.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/en_directo.h"

#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"
#include "thormang3_walking_module_msgs/ResultExo.h"

#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/SetJointFeedBackGain.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#define WALKING_TUNE
#ifdef WALKING_TUNE
#include "thormang3_walking_module_msgs/WalkingJointStatesStamped.h"
#endif

namespace thormang3
{

class OnlineWalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<OnlineWalkingModule>
{
public:
  OnlineWalkingModule();
  virtual ~OnlineWalkingModule();

  std::vector<robotis_framework::StepData> cur_step_data_array;

  double hip_kx = 0 ;
  double hip_ky = 0 ; 
  double an_kx = 0 ;
  double an_ky = 0 ;

  double r_leg_hip_y_ofst = 0 ;
  double r_leg_hip_r_ofst = 0 ;
  double r_leg_hip_p_ofst = 0 ;
  double r_leg_kn_p_ofst  = 0 ;
  double r_leg_an_p_ofst  = 0 ;
  double r_leg_an_r_ofst  = 0 ;

  double l_leg_hip_y_ofst = 0 ;
  double l_leg_hip_r_ofst = 0 ;
  double l_leg_hip_p_ofst = 0 ;
  double l_leg_kn_p_ofst  = 0 ;
  double l_leg_an_p_ofst  = 0 ;
  double l_leg_an_r_ofst  = 0 ;

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void modifyMotorExo ();
  void wtdExo (double *x , double *y ,double kx ,double ky);  
  void parseOffsetData(const std::string &path);
  void pubExoRes_ ();
  void pubExoRes2 ();
  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  double gyro_roll_, gyro_pitch_;
  double orientation_roll_, orientation_pitch_;
  double r_foot_fx_N_,  r_foot_fy_N_,  r_foot_fz_N_;
  double r_foot_Tx_Nm_, r_foot_Ty_Nm_, r_foot_Tz_Nm_;
  double l_foot_fx_N_,  l_foot_fy_N_,  l_foot_fz_N_;
  double l_foot_Tx_Nm_, l_foot_Ty_Nm_, l_foot_Tz_Nm_;

  thormang3_walking_module_msgs::ResultExo resultExoMsg;

private:
  void publishRobotPose(void);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
#ifdef WALKING_TUNE
  void publishWalkingTuningData();
#endif

  /* ROS Topic Callback Functions */
  bool enable_directo(thormang3_walking_module_msgs::en_directo::Request  &req,
                                thormang3_walking_module_msgs::en_directo::Response &res);
  void direct_motor_callback(const sensor_msgs::JointState::ConstPtr &msg);
  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void JohnnyFtLeftCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void JohnnyFtRightCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  /* ROS Service Callback Functions */
  bool setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
                                      thormang3_walking_module_msgs::SetBalanceParam::Response &res);

  bool setJointFeedBackGainServiceCallback(thormang3_walking_module_msgs::SetJointFeedBackGain::Request  &req,
                                           thormang3_walking_module_msgs::SetJointFeedBackGain::Response &res);

  bool getReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request  &req,
                                            thormang3_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request  &req,
                                   thormang3_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(thormang3_walking_module_msgs::StartWalking::Request  &req,
                                   thormang3_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request  &req,
                                thormang3_walking_module_msgs::IsRunning::Response &res);
  bool removeExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
                                             thormang3_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, thormang3_walking_module_msgs::StepData& des);

  void setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg);

  void updateBalanceParam();

  bool checkBalanceOnOff();

  void queueThread();

  void setJointFeedBackGain(thormang3_walking_module_msgs::JointFeedBackGain& msg);
  void updateJointFeedBackGain();

  void publishVisualize (std::vector<robotis_framework::StepData> msg);

  std::map<std::string, int> joint_name_to_index_;
  std::map<std::string, double> result3;
  //std::map<std::string, robotis_framework::DynamixelState *> result2;

  bool            gazebo_;
  int            directo;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    process_mutex_;

  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;
  Eigen::MatrixXd desired_matrix_g_to_cob_;
  Eigen::MatrixXd desired_matrix_g_to_rfoot_;
  Eigen::MatrixXd desired_matrix_g_to_lfoot_;

  bool previous_running_, present_running;

  /* ROS Topic Publish Functions */
  int r_foot_ft_publish_checker_;
  int l_foot_ft_publish_checker_;

  ros::Publisher exo_vis_pup_;
  ros::Publisher exo_vis_zmp_pup_;
  ros::Publisher footstep_vis_pub_;

  ros::Publisher cob_plot;
  ros::Publisher zmp_plot;

  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher pelvis_base_msg_pub_;
  ros::Publisher done_msg_pub_;
  ros::Publisher blnc;
  ros::Publisher ExoRes_;
  ros::Publisher ExoRes2;
#ifdef WALKING_TUNE
  ros::Publisher walking_joint_states_pub_;
  ros::Publisher imu_orientation_states_pub_;
  ros::Publisher ft_states_pub_;
  thormang3_walking_module_msgs::WalkingJointStatesStamped walking_joint_states_msg_;
#endif

  thormang3_walking_module_msgs::RobotPose  robot_pose_msg_;
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  Eigen::MatrixXd balance_update_polynomial_coeff_;


  bool joint_feedback_update_with_loop_;
  double joint_feedback_update_duration_;
  double joint_feedback_update_sys_time_;
  Eigen::MatrixXd joint_feedback_update_polynomial_coeff_;

  thormang3_walking_module_msgs::JointFeedBackGain previous_joint_feedback_gain_;
  thormang3_walking_module_msgs::JointFeedBackGain current_joint_feedback_gain_;
  thormang3_walking_module_msgs::JointFeedBackGain desired_joint_feedback_gain_;

  thormang3_walking_module_msgs::BalanceParam previous_balance_param_;
  thormang3_walking_module_msgs::BalanceParam current_balance_param_;
  thormang3_walking_module_msgs::BalanceParam desired_balance_param_;

};

}

#endif /* THORMANG3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_ */
